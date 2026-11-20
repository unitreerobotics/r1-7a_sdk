#include "arm_motor_bus.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
    std::string port = "/dev/ttyUSB3";
    int baud = kArmDefaultBaudrate;
    int rate_hz = 50;
    int probe_ms = 3000;
    int samples = 20;
    std::string out_path = kDefaultJointCalibPath;
    std::array<float, kArmJointCount> dir{};
    std::array<float, kArmJointCount> scale{};
    bool has_dir = false;
    bool has_scale = false;
    bool interactive = false;
    bool capture = true;
    std::string load_path;
};

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "Pose the arm at the desired URDF zero, then run this tool.\n"
        << "Formula:  urdf = dir * scale * motor + offset\n"
        << "  capture: offset = -dir * scale * motor_angle  (so urdf=0)\n"
        << "Options:\n"
        << "  --port PATH       serial device (default /dev/ttyUSB3)\n"
        << "  --baud N          baudrate (default " << kArmDefaultBaudrate << ")\n"
        << "  --rate N          command rate Hz (default 50)\n"
        << "  --probe-ms N      feedback probe timeout (default 3000)\n"
        << "  --samples N       average N motor readings (default 20)\n"
        << "  --dir CSV         per-joint dir ±1, e.g. 1,-1,1,1,1,1,1,1\n"
        << "  --scale CSV       per-joint scale >0, e.g. 1,1,1,1,1,1,1,1\n"
        << "  --out PATH        write calib file (default " << kDefaultJointCalibPath << ")\n"
        << "  --load PATH       load existing calib first\n"
        << "  --no-capture      do not recompute offset; only apply dir/scale/load\n"
        << "  --interactive     flip dirs / set scale / capture / save\n"
        << "  -h, --help\n"
        << "Interactive: d <i> | D CSV | k <i> <scale> | K CSV | c | p | s | q\n";
}

bool parseDirCsv(const std::string& csv, std::array<float, kArmJointCount>& dir) {
    std::stringstream ss(csv);
    std::string tok;
    int i = 0;
    while (std::getline(ss, tok, ',')) {
        if (i >= kArmJointCount) {
            return false;
        }
        const float v = std::stof(tok);
        if (std::fabs(v) < 1e-6f) {
            return false;
        }
        dir[i++] = (v < 0.f) ? -1.f : 1.f;
    }
    return i == kArmJointCount;
}

bool parseScaleCsv(const std::string& csv, std::array<float, kArmJointCount>& scale) {
    std::stringstream ss(csv);
    std::string tok;
    int i = 0;
    while (std::getline(ss, tok, ',')) {
        if (i >= kArmJointCount) {
            return false;
        }
        const float v = std::stof(tok);
        if (std::fabs(v) < 1e-6f) {
            return false;
        }
        scale[i++] = std::fabs(v);
    }
    return i == kArmJointCount;
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 0; i < kArmJointCount; ++i) {
        opt.dir[i] = 1.f;
        opt.scale[i] = 1.f;
    }
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            printUsage(argv[0]);
            return false;
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            opt.port = argv[++i];
        } else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            opt.baud = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--rate") == 0 && i + 1 < argc) {
            opt.rate_hz = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--probe-ms") == 0 && i + 1 < argc) {
            opt.probe_ms = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--samples") == 0 && i + 1 < argc) {
            opt.samples = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--dir") == 0 && i + 1 < argc) {
            if (!parseDirCsv(argv[++i], opt.dir)) {
                std::cerr << "Invalid --dir, need " << kArmJointCount
                          << " non-zero values\n";
                return false;
            }
            opt.has_dir = true;
        } else if (std::strcmp(argv[i], "--scale") == 0 && i + 1 < argc) {
            if (!parseScaleCsv(argv[++i], opt.scale)) {
                std::cerr << "Invalid --scale, need " << kArmJointCount
                          << " non-zero values\n";
                return false;
            }
            opt.has_scale = true;
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            opt.out_path = argv[++i];
        } else if (std::strcmp(argv[i], "--load") == 0 && i + 1 < argc) {
            opt.load_path = argv[++i];
        } else if (std::strcmp(argv[i], "--no-capture") == 0) {
            opt.capture = false;
        } else if (std::strcmp(argv[i], "--interactive") == 0) {
            opt.interactive = true;
        } else {
            std::cerr << "Unknown arg: " << argv[i] << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    if (opt.rate_hz <= 0) {
        opt.rate_hz = 50;
    }
    if (opt.samples < 1) {
        opt.samples = 1;
    }
    return true;
}

void sendZeroTorque(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].kp = 0.f;
        cmds[i].kd = 0.f;
        cmds[i].q = 0.f;
        cmds[i].dq = 0.f;
        cmds[i].tau = 0.f;
    }
    bus.step(cmds);
}

void sendStop(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 0;
        cmds[i].timed_out = false;
    }
    bus.step(cmds);
}

bool waitForFeedback(ArmMotorBus& bus, int rate_hz, int timeout_ms) {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next = clock::now();

    std::cout << "Probing feedback (zero-torque FOC)...\n";
    while (clock::now() < deadline) {
        sendZeroTorque(bus);
        if (bus.allFeedbackReady()) {
            std::cout << "Feedback ready for all joints.\n";
            return true;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    const auto ready = bus.feedbackReady();
    std::cerr << "Feedback incomplete:";
    for (int i = 0; i < kArmJointCount; ++i) {
        if (!ready[i]) {
            std::cerr << " j" << i;
        }
    }
    std::cerr << "\n";
    return false;
}

float motorAngleFromState(float urdf_q, const JointCalibration& cal) {
    const float d = (std::fabs(cal.dir) < 1e-6f) ? 1.f : cal.dir;
    const float s = (std::fabs(cal.scale) < 1e-6f) ? 1.f : std::fabs(cal.scale);
    return (urdf_q - cal.offset) / (d * s);
}

bool averageMotorAngles(ArmMotorBus& bus,
                        int rate_hz,
                        int samples,
                        std::array<float, kArmJointCount>& motor_q) {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    motor_q.fill(0.f);
    auto next = clock::now();

    for (int n = 0; n < samples; ++n) {
        sendZeroTorque(bus);
        const auto st = bus.states();
        const auto& cals = bus.jointCalibrations();
        for (int i = 0; i < kArmJointCount; ++i) {
            motor_q[i] += motorAngleFromState(st[i].q, cals[i]);
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    for (int i = 0; i < kArmJointCount; ++i) {
        motor_q[i] /= static_cast<float>(samples);
    }
    return true;
}

void applyCalib(ArmMotorBus& bus,
                const std::array<float, kArmJointCount>& dir,
                const std::array<float, kArmJointCount>& scale,
                const std::array<float, kArmJointCount>& offset) {
    for (int i = 0; i < kArmJointCount; ++i) {
        bus.setJointCalibration(i, dir[i], scale[i], offset[i]);
    }
}

void clearToIdentity(ArmMotorBus& bus) {
    for (int i = 0; i < kArmJointCount; ++i) {
        bus.setJointCalibration(i, 1.f, 1.f, 0.f);
    }
}

/** Capture: offset = -dir * scale * motor so current pose maps to URDF 0. */
void captureOffsets(ArmMotorBus& bus,
                    const Options& opt,
                    std::array<float, kArmJointCount>& dir,
                    std::array<float, kArmJointCount>& scale,
                    std::array<float, kArmJointCount>& offset) {
    clearToIdentity(bus);

    std::array<float, kArmJointCount> motor_q{};
    averageMotorAngles(bus, opt.rate_hz, opt.samples, motor_q);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Motor angles (avg of " << opt.samples << "):\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        offset[i] = -dir[i] * scale[i] * motor_q[i];
        std::cout << "  j" << i
                  << " motor=" << motor_q[i]
                  << " dir=" << dir[i]
                  << " scale=" << scale[i]
                  << " offset=" << offset[i] << "\n";
    }

    applyCalib(bus, dir, scale, offset);

    for (int n = 0; n < 5; ++n) {
        sendZeroTorque(bus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / opt.rate_hz));
    }
    const auto st = bus.states();
    std::cout << "After calib, URDF q (expect ~0):\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  j" << i << " q=" << st[i].q << "\n";
    }
}

void printCalib(const std::array<float, kArmJointCount>& dir,
                const std::array<float, kArmJointCount>& scale,
                const std::array<float, kArmJointCount>& offset) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Calibration table:\n";
    std::cout << "  joint  dir    scale       offset\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  " << i
                  << "      " << std::setw(4) << dir[i]
                  << "  " << scale[i]
                  << "  " << offset[i] << "\n";
    }
    std::cout << "C++ apply snippet:\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  bus.setJointCalibration(" << i << ", "
                  << dir[i] << "f, " << scale[i] << "f, " << offset[i] << "f);\n";
    }
}

bool saveCalib(const std::string& path,
               const std::array<float, kArmJointCount>& dir,
               const std::array<float, kArmJointCount>& scale,
               const std::array<float, kArmJointCount>& offset) {
    std::ofstream ofs(path);
    if (!ofs) {
        std::cerr << "Failed to write " << path << "\n";
        return false;
    }
    ofs << "# joint_calib: urdf = dir * scale * motor + offset\n";
    ofs << "# joint dir scale offset\n";
    ofs << std::fixed << std::setprecision(8);
    for (int i = 0; i < kArmJointCount; ++i) {
        ofs << i << " " << dir[i] << " " << scale[i] << " " << offset[i] << "\n";
    }
    std::cout << "Wrote " << path << "\n";
    return true;
}

bool loadCalib(const std::string& path,
               std::array<float, kArmJointCount>& dir,
               std::array<float, kArmJointCount>& scale,
               std::array<float, kArmJointCount>& offset) {
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "Failed to read " << path << "\n";
        return false;
    }
    std::string line;
    int loaded = 0;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream ss(line);
        int joint = -1;
        float d = 1.f;
        float a = 0.f;
        float b = 0.f;
        if (!(ss >> joint >> d >> a)) {
            continue;
        }
        if (joint < 0 || joint >= kArmJointCount) {
            continue;
        }
        dir[joint] = (d < 0.f) ? -1.f : 1.f;
        if (ss >> b) {
            scale[joint] = std::fabs(a);
            offset[joint] = b;
        } else {
            scale[joint] = 1.f;
            offset[joint] = a;
        }
        ++loaded;
    }
    if (loaded < kArmJointCount) {
        std::cerr << "Warning: loaded " << loaded << "/" << kArmJointCount
                  << " joints from " << path << "\n";
    } else {
        std::cout << "Loaded calib from " << path << "\n";
    }
    return loaded > 0;
}

void runInteractive(ArmMotorBus& bus,
                    Options& opt,
                    std::array<float, kArmJointCount>& dir,
                    std::array<float, kArmJointCount>& scale,
                    std::array<float, kArmJointCount>& offset) {
    std::cout
        << "Interactive calibration.\n"
        << "  Put the arm at URDF zero pose, then:\n"
        << "  d <i>       flip dir of joint i\n"
        << "  D CSV       set all dirs\n"
        << "  k <i> <s>   set scale of joint i\n"
        << "  K CSV       set all scales\n"
        << "  c           capture offsets (offset = -dir*scale*motor)\n"
        << "  p           print table + URDF states\n"
        << "  s           save to " << opt.out_path << "\n"
        << "  q           quit\n";

    applyCalib(bus, dir, scale, offset);
    std::string line;
    while (true) {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line)) {
            break;
        }
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        std::string cmd;
        ss >> cmd;
        if (cmd == "q" || cmd == "quit") {
            break;
        } else if (cmd == "d") {
            int j = -1;
            if (!(ss >> j) || j < 0 || j >= kArmJointCount) {
                std::cerr << "Usage: d <joint>\n";
                continue;
            }
            dir[j] = -dir[j];
            clearToIdentity(bus);
            std::array<float, kArmJointCount> motor_q{};
            averageMotorAngles(bus, opt.rate_hz, std::max(5, opt.samples / 4), motor_q);
            offset[j] = -dir[j] * scale[j] * motor_q[j];
            applyCalib(bus, dir, scale, offset);
            std::cout << "j" << j << " dir=" << dir[j]
                      << " scale=" << scale[j]
                      << " offset=" << offset[j] << "\n";
        } else if (cmd == "D") {
            std::string csv;
            if (!(ss >> csv) || !parseDirCsv(csv, dir)) {
                std::cerr << "Usage: D 1,-1,1,1,1,1,1\n";
                continue;
            }
            std::cout << "Dirs updated; run 'c' to recapture offsets.\n";
            printCalib(dir, scale, offset);
        } else if (cmd == "k") {
            int j = -1;
            float s = 0.f;
            if (!(ss >> j >> s) || j < 0 || j >= kArmJointCount || std::fabs(s) < 1e-6f) {
                std::cerr << "Usage: k <joint> <scale>\n";
                continue;
            }
            scale[j] = std::fabs(s);
            clearToIdentity(bus);
            std::array<float, kArmJointCount> motor_q{};
            averageMotorAngles(bus, opt.rate_hz, std::max(5, opt.samples / 4), motor_q);
            offset[j] = -dir[j] * scale[j] * motor_q[j];
            applyCalib(bus, dir, scale, offset);
            std::cout << "j" << j << " scale=" << scale[j]
                      << " offset=" << offset[j] << "\n";
        } else if (cmd == "K") {
            std::string csv;
            if (!(ss >> csv) || !parseScaleCsv(csv, scale)) {
                std::cerr << "Usage: K 1,1,1,1,1,1,1\n";
                continue;
            }
            std::cout << "Scales updated; run 'c' to recapture offsets.\n";
            printCalib(dir, scale, offset);
        } else if (cmd == "c") {
            captureOffsets(bus, opt, dir, scale, offset);
            printCalib(dir, scale, offset);
        } else if (cmd == "p") {
            printCalib(dir, scale, offset);
            sendZeroTorque(bus);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            const auto st = bus.states();
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Current URDF states:\n";
            for (int i = 0; i < kArmJointCount; ++i) {
                std::cout << "  j" << i << " q=" << st[i].q
                          << " dq=" << st[i].dq
                          << " tau=" << st[i].tau_est << "\n";
            }
        } else if (cmd == "s") {
            saveCalib(opt.out_path, dir, scale, offset);
        } else {
            std::cerr << "Unknown command. Use d/D/k/K/c/p/s/q\n";
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    try {
        if (!parseArgs(argc, argv, opt)) {
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Arg error: " << e.what() << "\n";
        return 1;
    }

    std::array<float, kArmJointCount> dir{};
    std::array<float, kArmJointCount> scale{};
    std::array<float, kArmJointCount> offset{};
    for (int i = 0; i < kArmJointCount; ++i) {
        dir[i] = 1.f;
        scale[i] = 1.f;
        offset[i] = 0.f;
    }

    if (!opt.load_path.empty()) {
        if (!loadCalib(opt.load_path, dir, scale, offset)) {
            return 1;
        }
    }

    ArmMotorBus bus;
    if (!bus.open(opt.port, opt.baud)) {
        return 1;
    }

    // Start from file loaded by open(); overlay --load / --dir / --scale.
    {
        const auto& cals = bus.jointCalibrations();
        for (int i = 0; i < kArmJointCount; ++i) {
            dir[i] = cals[i].dir;
            scale[i] = cals[i].scale;
            offset[i] = cals[i].offset;
        }
        if (!opt.load_path.empty()) {
            loadCalib(opt.load_path, dir, scale, offset);
        }
        if (opt.has_dir) {
            dir = opt.dir;
        }
        if (opt.has_scale) {
            scale = opt.scale;
        }
    }

    if (!waitForFeedback(bus, opt.rate_hz, opt.probe_ms)) {
        sendStop(bus);
        bus.close();
        return 1;
    }

    if (opt.interactive) {
        runInteractive(bus, opt, dir, scale, offset);
    } else {
        if (opt.capture) {
            captureOffsets(bus, opt, dir, scale, offset);
        } else {
            applyCalib(bus, dir, scale, offset);
            std::cout << "Applied without recapturing offsets.\n";
        }
        printCalib(dir, scale, offset);
        saveCalib(opt.out_path, dir, scale, offset);
    }

    std::cout << "Sending stop.\n";
    for (int n = 0; n < 10; ++n) {
        sendStop(bus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / opt.rate_hz));
    }
    bus.stop();
    bus.close();
    return 0;
}
