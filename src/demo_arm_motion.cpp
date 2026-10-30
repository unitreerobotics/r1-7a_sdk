#include "arm_motor_bus.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

using JointArray = std::array<float, kArmJointCount>;
using Clock = std::chrono::steady_clock;

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

// URDF joint limits (rad); index 0 = base joint (J1) ... 6 = wrist end (J7).
constexpr JointArray kLowerLimit = {-2.723f, 0.f, -0.349f, 0.f, -2.269f, -0.785f, -1.571f};
constexpr JointArray kUpperLimit = {2.723f, 3.1416f, 3.491f, 3.1415f, 2.269f, 0.785f, 1.571f};

constexpr float kPi = 3.14159265f;
constexpr int kFirstWristJoint = 4;  // joints 4..6 use --wrist-speed

enum class Hand { None, Right, Left };

struct Options {
    std::string port = "/dev/ttyUSB3";
    int baud = kArmDefaultBaudrate;
    float kp = 60.f;
    float kd = 1.0f;
    float speed = 4.0f;         // peak speed of joints 0..3 (rad/s)
    float wrist_speed = 6.0f;   // peak speed of joints 4..6 (rad/s)
    float limit_margin = 0.05f; // stay this far inside URDF limits (rad)
    float max_track_err = 0.6f; // abort if |q_cmd - q_meas| exceeds this (rad)
    int rate_hz = 100;
    int pause_ms = 150;
    int limit_pause_ms = 50;  // pause at min/max limit before reversing
    Hand hand = Hand::None;
};

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [-r|-l] [options]\n"
        << "Demo sequence (joint 0 = base ... joint 6 = wrist end):\n"
        << "  1. joint 0 -> -2.7 rad and joint 3 -> 1.0 rad together\n"
        << "  2. joint 6 -> min limit -> max limit -> 0\n"
        << "  3. joint 5 -> min limit -> max limit -> 0\n"
        << "  4. joint 4 -> min limit -> max limit -> 0\n"
        << "  5. joint 1 -> 1.5 and joint 3 -> 3.1 together\n"
        << "  6. joint 0 -> 0, joint 1 -> 0, joint 3 -> 1.0 together\n"
        // 7. joint 1 -> 1.5 and joint 3 -> 3.1 together (disabled)
        // 8. joint 2 -> 3.4 (disabled)
        // 9. joint 0 -> 2.7, joints 1/2 -> 0, joint 3 -> 1.0 together (disabled)
        << " 10. joint 0 -> 0 and joint 3 -> 0.5 together\n"
        << " 11. joint 4 -> max limit, then joint 3: 0.5 -> 2.0 -> 0.5, "
           "repeated 3 times\n"
        << " 12. all joints -> 0\n"
        << " 13. stop (mode 0) and exit\n"
        << "Hand (optional, default None):\n"
        << "  -r, --right       right arm (calib " << kRightJointCalibPath << ")\n"
        << "  -l, --left        left arm (calib " << kLeftJointCalibPath << ")\n"
        << "Options:\n"
        << "  --port PATH       serial device (default /dev/ttyUSB3)\n"
        << "  --baud N          baudrate (default " << kArmDefaultBaudrate << ")\n"
        << "  --kp F            position gain Nm/rad (default 60)\n"
        << "  --kd F            damping Nm/(rad/s) (default 1.0)\n"
        << "  --speed F         peak speed of joints 0-3 rad/s (default 4.0)\n"
        << "  --wrist-speed F   peak speed of joints 4-6 rad/s (default 6.0)\n"
        << "  --limit-margin F  margin inside URDF limits rad (default 0.05)\n"
        << "  --max-err F       abort on tracking error above F rad (default 0.6)\n"
        << "  --rate N          command rate Hz (default 100)\n"
        << "  --pause-ms N      pause between segments (default 150)\n"
        << "  --limit-pause-ms N  pause at joint 4/5/6 limits before reversing "
           "(default 50)\n";
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const bool has_val = i + 1 < argc;
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            printUsage(argv[0]);
            return false;
        } else if (std::strcmp(argv[i], "-r") == 0 || std::strcmp(argv[i], "--right") == 0) {
            if (opt.hand != Hand::None) {
                std::cerr << "Use only one of -r / -l\n";
                return false;
            }
            opt.hand = Hand::None;
        } else if (std::strcmp(argv[i], "-l") == 0 || std::strcmp(argv[i], "--left") == 0) {
            if (opt.hand != Hand::None) {
                std::cerr << "Use only one of -r / -l\n";
                return false;
            }
            opt.hand = Hand::Left;
        } else if (std::strcmp(argv[i], "--port") == 0 && has_val) {
            opt.port = argv[++i];
        } else if (std::strcmp(argv[i], "--baud") == 0 && has_val) {
            opt.baud = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--kp") == 0 && has_val) {
            opt.kp = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--kd") == 0 && has_val) {
            opt.kd = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--speed") == 0 && has_val) {
            opt.speed = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--wrist-speed") == 0 && has_val) {
            opt.wrist_speed = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--limit-margin") == 0 && has_val) {
            opt.limit_margin = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--max-err") == 0 && has_val) {
            opt.max_track_err = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--rate") == 0 && has_val) {
            opt.rate_hz = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--pause-ms") == 0 && has_val) {
            opt.pause_ms = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--limit-pause-ms") == 0 && has_val) {
            opt.limit_pause_ms = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown or incomplete arg: " << argv[i] << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    if (opt.rate_hz <= 0) {
        opt.rate_hz = 100;
    }
    if (opt.hand == Hand::None) {
        opt.hand = Hand::Right;  // default to right hand
    }
    opt.speed = std::max(0.01f, opt.speed);
    opt.wrist_speed = std::max(0.01f, opt.wrist_speed);
    opt.limit_margin = std::max(0.f, opt.limit_margin);
    return true;
}

void printStates(const std::array<ArmJointState, kArmJointCount>& s) {
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  j" << i << " q=" << s[i].q
                  << " dq=" << s[i].dq
                  << " tau=" << s[i].tau_est
                  << " mode=" << static_cast<int>(s[i].mode)
                  << " err=0x" << std::hex << s[i].error_code << std::dec << "\n";
    }
    std::cout << "\n";
}

void sendFoc(ArmMotorBus& bus, const JointArray& q, const JointArray& dq,
             float kp, float kd) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].q = q[i];
        cmds[i].dq = dq[i];
        cmds[i].tau = 0.f;
        cmds[i].kp = kp;
        cmds[i].kd = kd;
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

void sendZeroTorque(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
    }
    bus.step(cmds);
}

void shutdown(ArmMotorBus& bus, int rate_hz) {
    std::cout << "Sending stop (mode 0).\n";
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(500);
    auto next = Clock::now();
    while (Clock::now() < deadline) {
        sendStop(bus);
        next += period;
        std::this_thread::sleep_until(next);
    }
    bus.stop();
    printStates(bus.states());
    bus.close();
}

/** Motors only reply after a control packet; probe with zero-torque FOC. */
bool waitForFeedback(ArmMotorBus& bus, int rate_hz, int timeout_ms) {
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next = Clock::now();
    std::cout << "Probing feedback (zero-torque FOC)...\n";
    while (g_running && Clock::now() < deadline) {
        sendZeroTorque(bus);
        if (bus.allFeedbackReady()) {
            std::cout << "Feedback ready for all " << kArmJointCount << " joints.\n";
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

enum class StepResult { Ok, Fault, Interrupted };

class MotionDemo {
public:
    MotionDemo(ArmMotorBus& bus, const Options& opt) : bus_(bus), opt_(opt) {}

    /** Enable FOC holding the measured pose; succeeds once every joint reports mode 1. */
    bool enable(const JointArray& q0) {
        q_cmd_ = q0;
        const JointArray zero{};
        const auto period = std::chrono::microseconds(1000000 / opt_.rate_hz);
        const auto deadline = Clock::now() + std::chrono::milliseconds(1000);
        auto next = Clock::now();
        while (g_running && Clock::now() < deadline) {
            sendFoc(bus_, q_cmd_, zero, opt_.kp, opt_.kd);
            const auto st = bus_.states();
            bool all_foc = true;
            for (int i = 0; i < kArmJointCount; ++i) {
                if (st[i].mode != 1) {
                    all_foc = false;
                    break;
                }
            }
            if (all_foc) {
                return true;
            }
            next += period;
            std::this_thread::sleep_until(next);
        }
        const auto st = bus_.states();
        printStates(st);
        for (int i = 0; i < kArmJointCount; ++i) {
            if (st[i].mode != 1) {
                std::cerr << "Joint " << i << " not in FOC (mode="
                          << static_cast<int>(st[i].mode) << " err=0x" << std::hex
                          << st[i].error_code << std::dec
                          << "). Try --clear / --reset in test_arm_lowlevel.\n";
            }
        }
        return false;
    }

    float clampToLimits(int joint, float q) const {
        return std::min(kUpperLimit[joint], std::max(kLowerLimit[joint], q));
    }

    /**
     * Cosine-profile move of the joints in `targets` (joint, q) from the current
     * command; all listed joints start and finish together, others hold.
     */
    StepResult moveJoints(const std::vector<std::pair<int, float>>& targets,
                          const char* label,
                          int pause_ms,
                          float speed_override = 0.f) {
        const JointArray start = q_cmd_;
        JointArray goal = q_cmd_;
        float duration_s = 0.2f;
        for (const auto& t : targets) {
            const int j = t.first;
            goal[j] = clampToLimits(j, t.second);
            const float default_speed = (j >= kFirstWristJoint) ? opt_.wrist_speed : opt_.speed;
            const float speed = (speed_override > 0.f) ? speed_override : default_speed;
            // Peak speed of 0.5*(1-cos(pi*s)) is (pi/2) * delta / T.
            duration_s = std::max(duration_s,
                                  kPi * 0.5f * std::fabs(goal[j] - start[j]) / speed);
        }

        std::cout << std::fixed << std::setprecision(3) << "[move] " << label << ":";
        for (const auto& t : targets) {
            std::cout << " j" << t.first << " " << start[t.first] << " -> "
                      << goal[t.first];
        }
        std::cout << " (" << duration_s << " s)\n";

        const auto period = std::chrono::microseconds(1000000 / opt_.rate_hz);
        const auto t0 = Clock::now();
        auto next = t0;
        while (true) {
            if (!g_running) {
                return StepResult::Interrupted;
            }
            const float t = std::chrono::duration<float>(Clock::now() - t0).count();
            const float s = std::min(1.f, t / duration_s);
            const float blend = 0.5f * (1.f - std::cos(kPi * s));
            const float blend_rate =
                (s < 1.f) ? 0.5f * kPi * std::sin(kPi * s) / duration_s : 0.f;

            JointArray dq{};
            for (int i = 0; i < kArmJointCount; ++i) {
                const float delta = goal[i] - start[i];
                q_cmd_[i] = start[i] + delta * blend;
                dq[i] = delta * blend_rate;
            }
            sendFoc(bus_, q_cmd_, dq, opt_.kp, opt_.kd);
            if (!checkHealthy()) {
                return StepResult::Fault;
            }
            if (s >= 1.f) {
                break;
            }
            next += period;
            std::this_thread::sleep_until(next);
        }
        q_cmd_ = goal;
        return hold(pause_ms);
    }

    /** FOC hold at the current command for duration_ms. */
    StepResult hold(int duration_ms) {
        const JointArray zero{};
        const auto period = std::chrono::microseconds(1000000 / opt_.rate_hz);
        const auto t0 = Clock::now();
        auto next = t0;
        while (g_running) {
            if (Clock::now() - t0 >= std::chrono::milliseconds(duration_ms)) {
                return StepResult::Ok;
            }
            sendFoc(bus_, q_cmd_, zero, opt_.kp, opt_.kd);
            if (!checkHealthy()) {
                return StepResult::Fault;
            }
            next += period;
            std::this_thread::sleep_until(next);
        }
        return StepResult::Interrupted;
    }

private:
    bool checkHealthy() {
        const auto st = bus_.states();
        for (int i = 0; i < kArmJointCount; ++i) {
            if (st[i].mode != 1 || st[i].error_code != 0) {
                printStates(st);
                std::cerr << "Abort: joint " << i << " left FOC or faulted (mode="
                          << static_cast<int>(st[i].mode) << " err=0x" << std::hex
                          << st[i].error_code << std::dec << ").\n";
                return false;
            }
            const float err = std::fabs(q_cmd_[i] - st[i].q);
            if (err > opt_.max_track_err) {
                printStates(st);
                std::cerr << "Abort: joint " << i << " tracking error " << err
                          << " rad > --max-err " << opt_.max_track_err << ".\n";
                return false;
            }
        }
        return true;
    }

    ArmMotorBus& bus_;
    const Options& opt_;
    JointArray q_cmd_{};
};

StepResult runSequence(MotionDemo& demo, const Options& opt) {
    struct Segment {
        std::vector<std::pair<int, float>> targets;
        const char* label;
        int pause_ms;
        float speed_override = 0.f;  // >0: override per-segment speed (rad/s)
    };
    const auto lo = [&](int j) { return kLowerLimit[j] + opt.limit_margin; };
    const auto hi = [&](int j) { return kUpperLimit[j] - opt.limit_margin; };
    std::vector<std::pair<int, float>> all_zero;
    for (int i = 0; i < kArmJointCount; ++i) {
        all_zero.emplace_back(i, 0.f);
    }
    const int pause = opt.pause_ms;
    const int limit_pause = opt.limit_pause_ms;
    std::vector<Segment> segments = {
        {{{0, -2.7f}, {3, 1.0f}}, "joint 0 -> -2.7, joint 3 -> 1.0", pause},
        {{{6, lo(6)}}, "joint 6 -> min limit", limit_pause},
        {{{6, hi(6)}}, "joint 6 -> max limit", limit_pause},
        {{{6, 0.f}}, "joint 6 -> 0", pause},
        {{{5, lo(5)}}, "joint 5 -> min limit", limit_pause},
        {{{5, hi(5)}}, "joint 5 -> max limit", limit_pause},
        {{{5, 0.f}}, "joint 5 -> 0", pause},
        {{{4, lo(4)}}, "joint 4 -> min limit", limit_pause},
        {{{4, hi(4)}}, "joint 4 -> max limit", limit_pause},
        {{{4, 0.f}}, "joint 4 -> 0", pause},
        {{{1, 1.5f}, {3, 3.1f}}, "joint 1 -> 1.5, joint 3 -> 3.1", pause},
        {{{0, 0.f}, {1, 0.f}, {3, 1.0f}},
         "joint 0 -> 0, joint 1 -> 0, joint 3 -> 1.0", pause, opt.speed * 0.5f},
        // 7. joint 1 -> 1.5 and joint 3 -> 3.1 together (disabled)
        // {{{1, 1.5f}, {3, 3.1f}}, "joint 1 -> 1.5, joint 3 -> 3.1", pause},
        // 8. joint 2 -> 3.4 (disabled)
        // {{{2, 3.4f}}, "joint 2 -> 3.4", pause},
        // 9. joint 0 -> 2.7, joints 1/2 -> 0, joint 3 -> 1.0 together (disabled)
        // {{{0, 2.7f}, {1, 0.f}, {2, 0.f}, {3, 1.0f}},
        //  "joint 0 -> 2.7, joint 1 -> 0, joint 2 -> 0, joint 3 -> 1.0", pause},
        {{{0, 0.f}, {1, 0.2f}, {3, 0.5f}}, "joint 0 -> 0, joint 3 -> 0.5", pause},
        {{{4, hi(4)}}, "joint 4 -> max limit", pause},
    };
    constexpr float kJoint3ReturnSpeed = 2.0f;  // slower speed for step 11
    for (int n = 0; n < 3; ++n) {
        segments.push_back({{{3, 2.0f}}, "joint 3 -> 2.0", limit_pause, kJoint3ReturnSpeed});
        segments.push_back({{{3, 0.5f}}, "joint 3 -> 0.5", limit_pause, kJoint3ReturnSpeed});
    }
    segments.push_back({all_zero, "all joints -> 0", pause});
    for (const auto& seg : segments) {
        const StepResult rc = demo.moveJoints(seg.targets, seg.label, seg.pause_ms, seg.speed_override);
        if (rc != StepResult::Ok) {
            return rc;
        }
    }
    std::cout << "Sequence done at zero pose.\n";
    return StepResult::Ok;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    try {
        if (!parseArgs(argc, argv, opt)) {
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Bad argument: " << e.what() << std::endl;
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const std::string calib_path =
        (opt.hand == Hand::Left) ? kLeftJointCalibPath : kRightJointCalibPath;

    ArmMotorBus bus;
    if (!bus.open(opt.port, opt.baud, calib_path)) {
        return 1;
    }

    std::cout << "R1 arm motion demo\n"
              << "  hand=" << (opt.hand == Hand::Left ? "left" : "right")
              << " port=" << opt.port << " kp=" << opt.kp << " kd=" << opt.kd
              << " speed=" << opt.speed << " wrist_speed=" << opt.wrist_speed
              << " rad/s rate=" << opt.rate_hz << " Hz\n";

    if (!waitForFeedback(bus, opt.rate_hz, 3000)) {
        std::cerr << "Abort: no motor feedback.\n";
        shutdown(bus, opt.rate_hz);
        return 1;
    }

    const auto st0 = bus.states();
    JointArray q0{};
    for (int i = 0; i < kArmJointCount; ++i) {
        q0[i] = st0[i].q;
    }
    std::cout << "Start pose:\n";
    printStates(st0);

    MotionDemo demo(bus, opt);
    if (!demo.enable(q0)) {
        shutdown(bus, opt.rate_hz);
        return 1;
    }
    std::cout << "All joints in FOC.\n";
    // Let last FOC replies finish before the trajectory TX starts.
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    const StepResult rc = runSequence(demo, opt);
    g_running = true;
    shutdown(bus, opt.rate_hz);
    std::cout << "Done.\n";
    return rc == StepResult::Fault ? 1 : 0;
}
