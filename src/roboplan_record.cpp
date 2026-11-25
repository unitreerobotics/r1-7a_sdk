// Zero-gravity teach-and-play: hold the arm in zero-g, record a joint-space
// trajectory while the user moves it by hand, then replay the recorded motion.
//
// Workflow:
//   1. Enter zero-gravity compensation loop.
//   2. Press ENTER to start recording joint angles.
//   3. Move the arm by hand; data is sampled at --record-hz.
//   4. Press ENTER again to stop recording and save the CSV.
//   5. Press ENTER to replay, or 'q' + ENTER to quit.

#include "arm_motor_bus.h"

#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
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

constexpr const char* kDefaultUrdfPath =
    "../urdf/R1_Robotic_Arm_R_V1.2-1023/R1_Robotic_Arm_V1.2/R1_Robotic_Arm/urdf/"
    "R1_Robotic_Arm.urdf";

enum class Hand { None, Right, Left };

struct Options {
    std::string port = "/dev/ttyUSB3";
    int baud = kArmDefaultBaudrate;
    std::string urdf_path = kDefaultUrdfPath;
    std::string output_path = "/home/caoboxi/extra/R1_arm/output/r1-7a_sdk/csv/recorded_traj.csv";
    float kp = 60.f;
    float kd = 1.0f;
    float tau_limit = 15.f;     // gravity-comp torque clamp (Nm)
    float kd_gc = 0.3f;         // light damping in zero-g mode
    int rate_hz = 200;          // control rate for zero-g and replay
    int record_hz = 50;         // trajectory sampling rate
    int enable_timeout_ms = 3000; // max time to wait for all joints to enter FOC
    float max_track_err = 0.6f; // abort replay if tracking error exceeds this
    float speed_scale = 1.0f;   // replay speed scale
    int loop_count = 1;         // replay loops; -1 = forever
    Hand hand = Hand::None;
};

enum class UserCommand { None, Advance, Quit };

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [-r|-l] [options]\n"
        << "Zero-gravity teach-and-play.\n"
        << "Hand (required):\n"
        << "  -r, --right       right arm (calib " << kRightJointCalibPath << ")\n"
        << "  -l, --left        left arm (calib " << kLeftJointCalibPath << ")\n"
        << "Options:\n"
        << "  --port PATH       serial device (default /dev/ttyUSB3)\n"
        << "  --baud N          baudrate (default " << kArmDefaultBaudrate << ")\n"
        << "  --urdf PATH       URDF file (default " << kDefaultUrdfPath << ")\n"
        << "  -o PATH           output trajectory CSV (default "
        << "/home/caoboxi/extra/R1_arm/output/r1-7a_sdk/csv/recorded_traj.csv)\n"
        << "  --tau-limit F     gravity torque clamp Nm (default 15)\n"
        << "  --kd-gc F         zero-g damping Nm/(rad/s) (default 0.3)\n"
        << "  --rate N          control rate Hz (default 200)\n"
        << "  --record-hz N     trajectory sampling rate Hz (default 50)\n"
        << "  --enable-timeout N max ms to wait for FOC enable (default 3000)\n"
        << "  --kp F            replay position gain Nm/rad (default 60)\n"
        << "  --kd F            replay damping Nm/(rad/s) (default 1.0)\n"
        << "  --speed-scale F   replay speed scale (default 1.0)\n"
        << "  --loop N          replay loops; -1 = forever (default 1)\n"
        << "  --max-err F       abort replay on tracking error > F rad (default 0.6)\n"
        << "Ctrl-C stops all motors and exits.\n";
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
            opt.hand = Hand::Right;
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
        } else if (std::strcmp(argv[i], "--urdf") == 0 && has_val) {
            opt.urdf_path = argv[++i];
        } else if (std::strcmp(argv[i], "-o") == 0 && has_val) {
            opt.output_path = argv[++i];
        } else if (std::strcmp(argv[i], "--tau-limit") == 0 && has_val) {
            opt.tau_limit = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--kd-gc") == 0 && has_val) {
            opt.kd_gc = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--rate") == 0 && has_val) {
            opt.rate_hz = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--record-hz") == 0 && has_val) {
            opt.record_hz = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--enable-timeout") == 0 && has_val) {
            opt.enable_timeout_ms = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--kp") == 0 && has_val) {
            opt.kp = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--kd") == 0 && has_val) {
            opt.kd = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--speed-scale") == 0 && has_val) {
            opt.speed_scale = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--loop") == 0 && has_val) {
            opt.loop_count = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--max-err") == 0 && has_val) {
            opt.max_track_err = std::stof(argv[++i]);
        } else {
            std::cerr << "Unknown or incomplete arg: " << argv[i] << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    if (opt.hand == Hand::None) {
        std::cerr << "A hand is required: use -r / --right or -l / --left\n";
        printUsage(argv[0]);
        return false;
    }
    if (opt.rate_hz <= 0) opt.rate_hz = 200;
    if (opt.record_hz <= 0) opt.record_hz = 50;
    if (opt.record_hz > opt.rate_hz) opt.record_hz = opt.rate_hz;
    if (opt.enable_timeout_ms < 500) opt.enable_timeout_ms = 500;
    opt.tau_limit = std::max(0.f, opt.tau_limit);
    opt.kd_gc = std::max(0.f, opt.kd_gc);
    opt.max_track_err = std::max(0.f, opt.max_track_err);
    opt.speed_scale = std::max(0.01f, opt.speed_scale);
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

void sendStop(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 0;
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

bool waitForFeedback(ArmMotorBus& bus, int rate_hz, int timeout_ms) {
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next = Clock::now();
    std::cout << "Probing feedback (zero-torque FOC)...\n";
    while (g_running && Clock::now() < deadline) {
        std::array<ArmJointCmd, kArmJointCount> cmds{};
        for (int i = 0; i < kArmJointCount; ++i) {
            cmds[i].mode = 1;
            cmds[i].timed_out = false;
        }
        bus.step(cmds);
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
        if (!ready[i]) std::cerr << " j" << i;
    }
    std::cerr << "\n";
    return false;
}

// Reads stdin in a separate thread and signals user actions.
void inputThread(std::atomic<UserCommand>& cmd) {
    std::string line;
    while (g_running) {
        if (!std::getline(std::cin, line)) {
            cmd.store(UserCommand::Quit);
            g_running = false;
            break;
        }
        if (line.size() == 1 && (line[0] == 'q' || line[0] == 'Q')) {
            cmd.store(UserCommand::Quit);
            g_running = false;
        } else {
            cmd.store(UserCommand::Advance);
        }
    }
}

bool drainCommand(std::atomic<UserCommand>& cmd) {
    const UserCommand c = cmd.exchange(UserCommand::None);
    return c == UserCommand::Advance || c == UserCommand::Quit;
}

bool quitRequested(std::atomic<UserCommand>& cmd) {
    return cmd.load() == UserCommand::Quit;
}

struct Waypoint {
    float t = 0.f;
    JointArray q{};
};

bool saveTrajectory(const std::string& path, const std::vector<Waypoint>& traj) {
    std::error_code ec;
    std::filesystem::path p(path);
    std::filesystem::create_directories(p.parent_path(), ec);
    if (ec) {
        std::cerr << "Failed to create output directory " << p.parent_path()
                  << ": " << ec.message() << "\n";
        return false;
    }
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "Failed to open output file: " << path << "\n";
        return false;
    }
    f << "# t,q0,q1,q2,q3,q4,q5,q6\n";
    f << std::fixed << std::setprecision(4);
    for (const auto& wp : traj) {
        f << wp.t;
        for (int i = 0; i < kArmJointCount; ++i) {
            f << "," << wp.q[i];
        }
        f << "\n";
    }
    std::cout << "Saved " << traj.size() << " waypoints to " << path << "\n";
    return true;
}

// Send gravity-compensation commands with kp=0 so the arm can be moved by hand.
void sendZeroGravity(ArmMotorBus& bus,
                     const pinocchio::Model& model,
                     pinocchio::Data& data,
                     const Options& opt) {
    const auto st = bus.states();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(kArmJointCount);
    for (int i = 0; i < kArmJointCount; ++i) q[i] = st[i].q;
    const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(model, data, q);
    const Eigen::VectorXd tau_ff = g.cwiseMax(-opt.tau_limit).cwiseMin(opt.tau_limit);

    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].q = st[i].q;
        cmds[i].dq = 0.f;
        cmds[i].tau = static_cast<float>(tau_ff[i]);
        cmds[i].kp = 0.f;
        cmds[i].kd = opt.kd_gc;
    }
    bus.step(cmds);
}

// Zero-gravity loop. Blocks until the user advances (start/stop) or quits.
// On the first call records a single sample at t=0; on the second call it
// continues sampling until the user advances again, then returns.
bool teachAndRecord(ArmMotorBus& bus,
                    const pinocchio::Model& model,
                    pinocchio::Data& data,
                    const Options& opt,
                    std::atomic<UserCommand>& cmd,
                    std::vector<Waypoint>& traj) {
    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    const auto record_period = std::chrono::microseconds(1000000 / opt.record_hz);

    std::cout << "\n=== Zero-gravity mode ===\n"
              << "Move the arm by hand. Press ENTER to start recording, "
              << "or 'q' + ENTER to quit.\n";

    // Always send a clean stop frame before entering zero-gravity mode,
    // so the motor firmware has a defined transition from FOC to tau-only.
    std::cout << "Sending one stop frame before zero-gravity mode.\n";
    sendStop(bus);
    auto next = Clock::now();
    next += period;
    std::this_thread::sleep_until(next);

    // Wait for the user to start recording.
    while (g_running && !drainCommand(cmd)) {
        sendZeroGravity(bus, model, data, opt);
        next += period;
        std::this_thread::sleep_until(next);
    }
    if (!g_running || quitRequested(cmd)) return false;

    // Recording phase.
    std::cout << "Recording... Press ENTER to stop.\n";
    traj.clear();
    const auto t0 = Clock::now();
    auto next_record = t0;
    while (g_running && !drainCommand(cmd)) {
        sendZeroGravity(bus, model, data, opt);

        const auto st = bus.states();
        if (Clock::now() >= next_record) {
            Waypoint wp;
            wp.t = std::chrono::duration<float>(Clock::now() - t0).count();
            for (int i = 0; i < kArmJointCount; ++i) wp.q[i] = st[i].q;
            traj.push_back(wp);
            next_record += record_period;
        }

        next += period;
        std::this_thread::sleep_until(next);
    }

    if (!g_running || quitRequested(cmd)) return false;
    if (traj.size() < 2) {
        std::cerr << "Recorded trajectory is too short (" << traj.size() << " waypoints).\n";
        return false;
    }
    std::cout << "Recorded " << traj.size() << " waypoints over "
              << std::fixed << std::setprecision(2) << traj.back().t << " s.\n";
    return true;
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

// Hold current position with FOC until every joint reports mode 1.
// Sends one stop frame first, then switches to FOC, because some motor
// firmware requires a clean stop->FOC transition after zero-torque mode.
bool enableFoc(ArmMotorBus& bus, const Options& opt) {
    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    const auto timeout = std::chrono::milliseconds(opt.enable_timeout_ms);
    auto next = Clock::now();

    // std::cout << "Clearing motor faults.\n";
    // bus.clearFaults();

    std::cout << "Sending one stop frame before enabling FOC.\n";
    sendStop(bus);
    next += period;
    // std::this_thread::sleep_until(next);

    bool cleared_once = false;
    auto phase_deadline = Clock::now() + timeout;
    std::cout << "Enabling FOC position mode (timeout " << opt.enable_timeout_ms << " ms)...\n";

    while (g_running) {
        const auto st = bus.states();
        JointArray q_hold{}, zero{};
        for (int i = 0; i < kArmJointCount; ++i) q_hold[i] = st[i].q;
        sendFoc(bus, q_hold, zero, opt.kp, opt.kd);

        bool all_foc = true;
        bool any_fault = false;
        for (int i = 0; i < kArmJointCount; ++i) {
            if (st[i].mode != 1) all_foc = false;
            if (st[i].error_code != 0) any_fault = true;
        }

        if (all_foc) {
            std::cout << "All joints in FOC.\n";
            return true;
        }

        // If faults appeared, try clearing once and restart the timeout.
        if (any_fault && !cleared_once) {
            std::cout << "Faults detected during enable; clearing and retrying.\n";
            bus.clearFaults();
            cleared_once = true;
            phase_deadline = Clock::now() + timeout;
            next = Clock::now();
            continue;
        }

        if (Clock::now() >= phase_deadline) {
            printStates(st);
            std::cerr << "Abort: not all joints entered FOC within "
                      << opt.enable_timeout_ms << " ms.\n";
            return false;
        }

        next += period;
        std::this_thread::sleep_until(next);
    }
    return false;
}

// Smooth cosine move from the current measured pose to q_start.
bool moveToStart(ArmMotorBus& bus, const JointArray& q_start, const Options& opt) {
    if (!enableFoc(bus, opt)) {
        return false;
    }

    const auto st0 = bus.states();
    JointArray q0{};
    for (int i = 0; i < kArmJointCount; ++i) q0[i] = st0[i].q;

    constexpr float kMoveSpeed = 0.3f;  // rad/s, conservative
    float max_delta = 0.f;
    for (int i = 0; i < kArmJointCount; ++i) {
        max_delta = std::max(max_delta, std::fabs(q_start[i] - q0[i]));
    }
    const float duration_s = std::max(0.5f, max_delta / kMoveSpeed);
    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    const auto t0 = Clock::now();
    auto next = t0;

    // Allow a short transient where a joint may report mode != 1.
    int unhealthy_ticks = 0;
    constexpr int kMaxUnhealthyTicks = 40;  // 200 ms at 200 Hz

    std::cout << "Moving to trajectory start over " << duration_s << " s...\n";
    while (g_running) {
        const float t = std::chrono::duration<float>(Clock::now() - t0).count();
        const float s = std::min(1.f, t / duration_s);
        const float blend = 0.5f * (1.f - std::cos(3.14159265f * s));
        const float blend_rate = (s < 1.f)
                                     ? 0.5f * 3.14159265f * std::sin(3.14159265f * s) / duration_s
                                     : 0.f;
        JointArray q_cmd{}, dq_cmd{};
        for (int i = 0; i < kArmJointCount; ++i) {
            const float delta = q_start[i] - q0[i];
            q_cmd[i] = q0[i] + delta * blend;
            dq_cmd[i] = delta * blend_rate;
        }
        sendFoc(bus, q_cmd, dq_cmd, opt.kp, opt.kd);

        const auto st = bus.states();
        bool healthy = true;
        for (int i = 0; i < kArmJointCount; ++i) {
            if (st[i].mode != 1 || st[i].error_code != 0) {
                healthy = false;
                break;
            }
        }
        if (!healthy) {
            ++unhealthy_ticks;
            if (unhealthy_ticks > kMaxUnhealthyTicks) {
                printStates(st);
                std::cerr << "Abort: joints left FOC or faulted for too long.\n";
                return false;
            }
        } else {
            unhealthy_ticks = 0;
        }

        if (s >= 1.f) break;
        next += period;
        std::this_thread::sleep_until(next);
    }
    std::cout << "At start pose.\n";
    return true;
}

void sampleTrajectory(const std::vector<Waypoint>& traj, float t, JointArray& q, JointArray& dq) {
    const float t0 = traj.front().t;
    const float te = traj.back().t;
    if (t <= t0) { q = traj.front().q; dq = {}; return; }
    if (t >= te) { q = traj.back().q; dq = {}; return; }
    size_t i = 0;
    for (size_t k = 0; k + 1 < traj.size(); ++k) {
        if (t >= traj[k].t && t <= traj[k + 1].t) { i = k; break; }
    }
    const float dt = traj[i + 1].t - traj[i].t;
    if (dt <= 0.f) { q = traj[i].q; dq = {}; return; }
    const float alpha = (t - traj[i].t) / dt;
    for (int j = 0; j < kArmJointCount; ++j) {
        const float q0 = traj[i].q[j];
        const float q1 = traj[i + 1].q[j];
        q[j] = q0 + alpha * (q1 - q0);
        dq[j] = (q1 - q0) / dt;
    }
}

enum class RunResult { Ok, Fault, Interrupted };

RunResult playTrajectory(ArmMotorBus& bus,
                         const std::vector<Waypoint>& traj,
                         const Options& opt) {
    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    const float duration = traj.back().t;
    const float speed_scale = opt.speed_scale;

    int loop_done = 0;
    // Allow short transient mode errors during playback.
    int unhealthy_ticks = 0;
    constexpr int kMaxUnhealthyTicks = 40;  // 200 ms at 200 Hz

    while (g_running && (opt.loop_count < 0 || loop_done < opt.loop_count)) {
        const auto loop_start = Clock::now();
        auto next = loop_start;
        ++loop_done;
        std::cout << "\n=== Replay loop " << loop_done << " / "
                  << (opt.loop_count < 0 ? "inf" : std::to_string(opt.loop_count))
                  << " ===\n";

        while (g_running) {
            const float elapsed_s =
                std::chrono::duration<float>(Clock::now() - loop_start).count();
            const float t = elapsed_s * speed_scale;

            JointArray q_cmd{}, dq_cmd{};
            sampleTrajectory(traj, t, q_cmd, dq_cmd);
            sendFoc(bus, q_cmd, dq_cmd, opt.kp, opt.kd);

            const auto st = bus.states();
            bool healthy = true;
            for (int i = 0; i < kArmJointCount; ++i) {
                if (st[i].mode != 1 || st[i].error_code != 0) {
                    healthy = false;
                    break;
                }
                const float err = std::fabs(q_cmd[i] - st[i].q);
                if (err > opt.max_track_err) {
                    printStates(st);
                    std::cerr << "Abort: joint " << i << " tracking error " << err
                              << " rad > --max-err " << opt.max_track_err << ".\n";
                    return RunResult::Fault;
                }
            }
            if (!healthy) {
                ++unhealthy_ticks;
                if (unhealthy_ticks > kMaxUnhealthyTicks) {
                    printStates(st);
                    std::cerr << "Abort: joints left FOC or faulted for too long.\n";
                    return RunResult::Fault;
                }
            } else {
                unhealthy_ticks = 0;
            }

            if (t >= duration) break;
            next += period;
            std::this_thread::sleep_until(next);
        }

        // Brief hold at the final pose between loops.
        if (g_running && (opt.loop_count < 0 || loop_done < opt.loop_count)) {
            const auto hold_end = Clock::now() + std::chrono::milliseconds(300);
            auto next_hold = Clock::now();
            const JointArray zero{};
            while (g_running && Clock::now() < hold_end) {
                sendFoc(bus, traj.back().q, zero, opt.kp, opt.kd);
                next_hold += period;
                std::this_thread::sleep_until(next_hold);
            }
        }
    }
    return g_running ? RunResult::Ok : RunResult::Interrupted;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            printUsage(argv[0]);
            return 0;
        }
    }

    Options opt;
    try {
        if (!parseArgs(argc, argv, opt)) {
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Bad argument: " << e.what() << std::endl;
        return 1;
    }

    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);

    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(opt.urdf_path, model);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load URDF '" << opt.urdf_path << "': " << e.what() << std::endl;
        return 1;
    }
    if (model.nq != kArmJointCount || model.nv != kArmJointCount) {
        std::cerr << "Unexpected model size: nq=" << model.nq << " nv=" << model.nv
                  << " (expected " << kArmJointCount << ").\n";
        return 1;
    }
    pinocchio::Data data(model);
    std::cout << "Loaded Pinocchio model '" << model.name << "' (nq=" << model.nq << ")\n";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const std::string calib_path =
        (opt.hand == Hand::Left) ? kLeftJointCalibPath : kRightJointCalibPath;

    ArmMotorBus bus;
    if (!bus.open(opt.port, opt.baud, calib_path)) {
        return 1;
    }

    std::cout << "R1 arm teach-and-play\n"
              << "  hand=" << (opt.hand == Hand::Left ? "left" : "right")
              << " port=" << opt.port << " rate=" << opt.rate_hz << " Hz\n";

    if (!waitForFeedback(bus, opt.rate_hz, 3000)) {
        std::cerr << "Abort: no motor feedback.\n";
        shutdown(bus, opt.rate_hz);
        return 1;
    }

    std::atomic<UserCommand> cmd{UserCommand::None};
    std::thread input_thr(inputThread, std::ref(cmd));

    int exit_code = 0;
    std::vector<Waypoint> traj;

    while (g_running) {
        const bool recorded = teachAndRecord(bus, model, data, opt, cmd, traj);
        if (!recorded) {
            if (g_running) {
                std::cout << "Recording cancelled.\n";
            }
            break;
        }

        if (!saveTrajectory(opt.output_path, traj)) {
            exit_code = 1;
            break;
        }

        std::cout << "\nStarting replay automatically.\n";
        if (!moveToStart(bus, traj.front().q, opt)) {
            exit_code = 1;
            break;
        }

        std::cout << "Replaying...\n";
        const RunResult rc = playTrajectory(bus, traj, opt);
        if (rc == RunResult::Fault) {
            exit_code = 1;
            break;
        }

        std::cout << "\n=== Replay finished. Back to zero-g mode ===\n"
                  << "Press ENTER to record another trajectory, "
                  << "or 'q' + ENTER to quit.\n";

        // Actually run zero-gravity compensation while waiting for user input.
        sendStop(bus);
        auto next = Clock::now();
        next += period;
        std::this_thread::sleep_until(next);
        while (g_running && !drainCommand(cmd)) {
            sendZeroGravity(bus, model, data, opt);
            next += period;
            std::this_thread::sleep_until(next);
        }
        if (!g_running || quitRequested(cmd)) {
            break;
        }
    }

    g_running = false;
    cmd.store(UserCommand::Quit);
    input_thr.join();
    shutdown(bus, opt.rate_hz);
    std::cout << "Done.\n";
    return exit_code;
}
