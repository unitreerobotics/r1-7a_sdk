#include "arm_motor_bus.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

enum class RunMode { None, Foc, Stop, ZeroTorque, Clear, Reset, Home, Goto, GotoHold };
enum class Hand { None, Right, Left };

struct Options {
    std::string port = "/dev/ttyUSB0";
    int baud = kArmDefaultBaudrate;
    float kp = 60.f;
    float kd = 0.5f;
    float home_lead = 0.2f;  // max |q_cmd - q_meas| while homing / goto (rad)
    int rate_hz = 50;
    int hold_ms = 0;  // FOC hold after reaching target; 0 = stop immediately
    int move_ms = 3000;
    RunMode mode = RunMode::None;
    Hand hand = Hand::None;
    std::array<float, kArmJointCount> q_target{};
    bool q_target_set = false;
};

const char* kExclusiveCmdHint =
    "Use only one of --foc / --stop / --zero-torque / --clear / --reset / "
    "--home / --goto / --goto-hold\n";

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " <command> [-r|-l] [options]\n"
        << "Commands (exactly one required):\n"
        << "  --foc           FOC hold current pose until Ctrl-C\n"
        << "  --stop          send mode 0 stop, then exit\n"
        << "  --zero-torque   FOC with kp=kd=tau=0 until Ctrl-C\n"
        << "  --clear         clear motor faults, then exit\n"
        << "  --reset         reset motors, then exit\n"
        << "  --home          FOC move all joints to q=0, then stop\n"
        << "  --goto          FOC move to --q targets, then stop\n"
        << "  --goto-hold     FOC move to --q targets, then hold until Ctrl-C\n"
        << "Hand (optional, default None):\n"
        << "  -r, --right     right arm (calib " << kRightJointCalibPath << ")\n"
        << "  -l, --left      left arm (calib " << kLeftJointCalibPath << ")\n"
        << "Options:\n"
        << "  --port PATH     serial device (default /dev/ttyUSB3)\n"
        << "  --baud N        baudrate (default " << kArmDefaultBaudrate << ")\n"
        << "  --kp F          position gain Nm/rad (default 40; home/goto torque ~ kp*lead)\n"
        << "  --kd F          damping Nm/(rad/s) (default 0.5)\n"
        << "  --q F0 .. F6    target joint positions rad (URDF; required by --goto)\n"
        << "  --home-lead F   max cmd lead while --home/--goto (default 0.2 rad)\n"
        << "  --rate N        command rate Hz (default 50)\n"
        << "  --hold-ms N     FOC hold at target after home/goto (default 0 = end now)\n"
        << "  --move-ms N     home / goto timeout (default 3000; auto-extends)\n"
        << "Ctrl-C sends stop (mode 0) and exits "
           "(--foc / --zero-torque / --home / --goto).\n";
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        auto next = [&](float& dst) {
            if (i + 1 >= argc) {
                return false;
            }
            dst = std::stof(argv[++i]);
            return true;
        };
        auto next_i = [&](int& dst) {
            if (i + 1 >= argc) {
                return false;
            }
            dst = std::stoi(argv[++i]);
            return true;
        };

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
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            opt.port = argv[++i];
        } else if (std::strcmp(argv[i], "--baud") == 0) {
            if (!next_i(opt.baud)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--kp") == 0) {
            if (!next(opt.kp)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--kd") == 0) {
            if (!next(opt.kd)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--home-lead") == 0) {
            if (!next(opt.home_lead)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--rate") == 0) {
            if (!next_i(opt.rate_hz)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--hold-ms") == 0) {
            if (!next_i(opt.hold_ms)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--move-ms") == 0) {
            if (!next_i(opt.move_ms)) {
                return false;
            }
        } else if (std::strcmp(argv[i], "--q") == 0) {
            if (i + kArmJointCount >= argc) {
                std::cerr << "--q needs " << kArmJointCount
                          << " joint positions (rad)\n";
                return false;
            }
            for (int j = 0; j < kArmJointCount; ++j) {
                opt.q_target[j] = std::stof(argv[++i]);
            }
            opt.q_target_set = true;
        } else if (std::strcmp(argv[i], "--foc") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::Foc;
        } else if (std::strcmp(argv[i], "--stop") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::Stop;
        } else if (std::strcmp(argv[i], "--zero-torque") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::ZeroTorque;
        } else if (std::strcmp(argv[i], "--clear") == 0 ||
                   std::strcmp(argv[i], "--clear-fault") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::Clear;
        } else if (std::strcmp(argv[i], "--reset") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::Reset;
        } else if (std::strcmp(argv[i], "--home") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::Home;
        } else if (std::strcmp(argv[i], "--goto") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::Goto;
        } else if (std::strcmp(argv[i], "--goto-hold") == 0) {
            if (opt.mode != RunMode::None) {
                std::cerr << kExclusiveCmdHint;
                return false;
            }
            opt.mode = RunMode::GotoHold;
        } else {
            std::cerr << "Unknown arg: " << argv[i] << std::endl;
            printUsage(argv[0]);
            return false;
        }
    }
    if (opt.mode == RunMode::None) {
        std::cerr << "A command is required.\n" << kExclusiveCmdHint;
        printUsage(argv[0]);
        return false;
    }
    if (opt.hand == Hand::None) {
        opt.hand = Hand::Right;  // default to right hand
    }
    if (opt.rate_hz <= 0) {
        opt.rate_hz = 50;
    }
    if (opt.home_lead < 0.01f) {
        opt.home_lead = 0.01f;
    }
    if ((opt.mode == RunMode::Goto || opt.mode == RunMode::GotoHold) &&
        !opt.q_target_set) {
        std::cerr << "--goto / --goto-hold requires --q with " << kArmJointCount
                  << " joint positions\n";
        return false;
    }
    if (opt.q_target_set && opt.mode != RunMode::Goto &&
        opt.mode != RunMode::GotoHold) {
        std::cerr << "--q is only valid with --goto / --goto-hold\n";
        return false;
    }
    return true;
}

void printStates(const std::array<ArmJointState, kArmJointCount>& s) {
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  j" << i << " q=" << s[i].q
                  << " dq=" << s[i].dq
                  << " tau=" << s[i].tau_est
                  << " mode=" << static_cast<int>(s[i].mode)
                  << " to=" << static_cast<int>(s[i].timeout)
                  << " err=0x" << std::hex << s[i].error_code << std::dec;
        if (i + 1 < kArmJointCount) {
            std::cout << "\n";
        }
    }
    std::cout << "\n";
    std::cout << "\n";
}

void printMotorStates(const std::array<ArmJointState, kArmJointCount>& s) {
    std::cout << "Motor-frame feedback (raw, no calib):\n";
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  m" << i << " q=" << s[i].q
                  << " dq=" << s[i].dq
                  << " tau=" << s[i].tau_est
                  << " mode=" << static_cast<int>(s[i].mode)
                  << " to=" << static_cast<int>(s[i].timeout)
                  << " err=0x" << std::hex << s[i].error_code << std::dec;
        if (i + 1 < kArmJointCount) {
            std::cout << "\n";
        }
    }
    std::cout << "\n\n";
}

void sendFocPosition(ArmMotorBus& bus,
                     const std::array<float, kArmJointCount>& q,
                     float kp,
                     float kd) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].q = q[i];
        cmds[i].dq = 0.f;
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
        cmds[i].q = 0.f;
        cmds[i].dq = 0.f;
        cmds[i].tau = 0.f;
        cmds[i].kp = 0.f;
        cmds[i].kd = 0.f;
    }
    bus.step(cmds);
}

void sendZeroTorque(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
        cmds[i].q = 0.f;
        cmds[i].dq = 0.f;
        cmds[i].tau = 0.f;
        cmds[i].kp = 0.f;
        cmds[i].kd = 0.f;
    }
    bus.step(cmds);
}

bool runFor(ArmMotorBus& bus,
            int duration_ms,
            int rate_hz,
            const std::function<void(float)>& tick) {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto t0 = clock::now();
    auto next = t0;
    const float dur_s = duration_ms / 1000.f;

    while (g_running) {
        const float t = std::chrono::duration<float>(clock::now() - t0).count();
        if (t * 1000.f >= static_cast<float>(duration_ms)) {
            break;
        }
        const float alpha = (dur_s > 0.f) ? (t / dur_s) : 1.f;
        tick(alpha);
        next += period;
        std::this_thread::sleep_until(next);
    }
    return g_running.load();
}

void runUntilStopped(ArmMotorBus& bus,
                     int rate_hz,
                     const std::function<void()>& tick) {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    auto next = clock::now();
    auto last_print = next;
    while (g_running) {
        tick();
        if (clock::now() - last_print >= std::chrono::seconds(1)) {
            printStates(bus.states());
            last_print = clock::now();
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
}

void shutdown(ArmMotorBus& bus, int rate_hz) {
    std::cout << "Sending stop (mode 0).\n";
    g_running = true;
    runFor(bus, 500, rate_hz, [&](float) { sendStop(bus); });
    bus.stop();
    printStates(bus.states());
    bus.close();
}

int estimateCreepTimeoutMs(float max_abs_delta, int move_ms) {
    constexpr float kCreepSpeed = 0.08f;
    int timeout_ms = move_ms;
    if (max_abs_delta > 1e-3f) {
        const int min_ms =
            static_cast<int>((max_abs_delta / kCreepSpeed) * 1000.f) + 5000;
        if (timeout_ms < min_ms) {
            timeout_ms = min_ms;
        }
    }
    return timeout_ms;
}

/** Lead-limited FOC creep toward q_target. Returns 0 ok, 1 fault, 2 interrupted. */
int creepToTargets(ArmMotorBus& bus,
                   const std::array<float, kArmJointCount>& q_target,
                   float kp,
                   float kd,
                   float lead,
                   int timeout_ms,
                   int rate_hz) {
    constexpr float kDoneRad = 0.07f;
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next = clock::now();
    auto last_print = next;
    bool reached = false;
    bool faulted = false;

    while (g_running && clock::now() < deadline) {
        const auto meas = bus.states();
        for (int i = 0; i < kArmJointCount; ++i) {
            if (meas[i].mode != 1 || meas[i].error_code != 0) {
                faulted = true;
                break;
            }
        }
        if (faulted) {
            break;
        }

        std::array<float, kArmJointCount> q{};
        bool all_near = true;
        for (int i = 0; i < kArmJointCount; ++i) {
            const float cur = meas[i].q;
            const float diff = q_target[i] - cur;
            if (std::fabs(diff) > kDoneRad) {
                all_near = false;
            }
            const float step = std::max(-lead, std::min(lead, diff));
            q[i] = cur + step;
        }
        sendFocPosition(bus, q, kp, kd);

        if (all_near) {
            reached = true;
            break;
        }
        if (clock::now() - last_print >= std::chrono::seconds(2)) {
            printStates(bus.states());
            last_print = clock::now();
        }
        next += period;
        std::this_thread::sleep_until(next);
    }

    printStates(bus.states());
    if (faulted) {
        std::cerr << "Creep aborted: left FOC or fault "
                     "(check err=0x8 undervoltage).\n";
        return 1;
    }
    if (!g_running) {
        return 2;
    }
    if (!reached) {
        std::cerr << "Creep timeout: not all joints near target "
                     "(increase --kp / --home-lead, or check gravity/PSU).\n";
    }
    return 0;
}

/** FOC hold -> creep to q_target -> hold -> stop. */
int runCreepSequence(ArmMotorBus& bus,
                     const Options& opt,
                     const std::array<float, kArmJointCount>& q0,
                     const std::array<float, kArmJointCount>& q_target,
                     const char* label,
                     bool hold_forever = false) {
    float max_abs = 0.f;
    for (int i = 0; i < kArmJointCount; ++i) {
        max_abs = std::max(max_abs, std::fabs(q_target[i] - q0[i]));
    }
    const float lead = opt.home_lead;
    const int creep_ms = estimateCreepTimeoutMs(max_abs, opt.move_ms);

    std::cout << "Command: " << label << "\n"
              << "  kp=" << opt.kp << " kd=" << opt.kd
              << " lead=" << lead << " rad"
              << " (~tau<=" << (opt.kp * lead) << " Nm)"
              << " timeout_ms=" << creep_ms
              << " (max|dq|=" << max_abs << " rad)\n"
              << "  target q:";
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << " " << q_target[i];
    }
    std::cout << "\n";

    printMotorStates(bus.motorStates());

    // Enable FOC at start pose; proceed as soon as all joints report mode==1.
    constexpr int kFocEnableTimeoutMs = 1000;
    std::cout << "Phase 1: FOC enable until all joints mode=1 "
              << "(timeout " << kFocEnableTimeoutMs << " ms)\n";
    {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
        const auto deadline =
            clock::now() + std::chrono::milliseconds(kFocEnableTimeoutMs);
        auto next = clock::now();
        bool all_foc = false;
        while (g_running && clock::now() < deadline) {
            sendFocPosition(bus, q0, opt.kp, opt.kd);
            const auto st = bus.states();
            all_foc = true;
            for (int i = 0; i < kArmJointCount; ++i) {
                if (st[i].mode != 1) {
                    all_foc = false;
                    break;
                }
            }
            if (all_foc) {
                break;
            }
            next += period;
            std::this_thread::sleep_until(next);
        }
        if (!g_running) {
            shutdown(bus, opt.rate_hz);
            std::cout << "Done.\n";
            return 0;
        }
        auto after_enable = bus.states();
        printStates(after_enable);
        if (!all_foc) {
            for (int i = 0; i < kArmJointCount; ++i) {
                if (after_enable[i].mode != 1) {
                    std::cerr << "Abort: joint " << i << " not in FOC (mode="
                              << static_cast<int>(after_enable[i].mode)
                              << " to=" << static_cast<int>(after_enable[i].timeout)
                              << " err=0x" << std::hex << after_enable[i].error_code
                              << std::dec << "). Try --clear / --reset or power-cycle.\n";
                    break;
                }
            }
            sendStop(bus);
            bus.stop();
            bus.close();
            return 1;
        }
        std::cout << "All joints in FOC, start move.\n";
    }

    // Let last FOC replies finish before Phase 2 TX (avoids a systematic CRC at boundary).
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    std::cout << "Phase 2: FOC creep to target (timeout " << creep_ms << " ms)\n";
    const int creep_rc =
        creepToTargets(bus, q_target, opt.kp, opt.kd, lead, creep_ms, opt.rate_hz);
    if (creep_rc != 0) {
        sendStop(bus);
        bus.stop();
        bus.close();
        std::cout << "Done.\n";
        return creep_rc == 2 ? 0 : 1;
    }

    if (hold_forever) {
        std::cout << "Phase 3: FOC hold target until Ctrl-C\n";
        runUntilStopped(bus, opt.rate_hz, [&]() {
            sendFocPosition(bus, q_target, opt.kp, opt.kd);
        });
        shutdown(bus, opt.rate_hz);
        std::cout << "Done.\n";
        return 0;
    }

    if (opt.hold_ms > 0) {
        std::cout << "Phase 3: FOC hold target (" << opt.hold_ms << " ms)\n";
        runFor(bus, opt.hold_ms, opt.rate_hz, [&](float) {
            sendFocPosition(bus, q_target, opt.kp, opt.kd);
        });
        printStates(bus.states());
    } else {
        std::cout << "Phase 3: skip hold at target (--hold-ms 0)\n";
        printStates(bus.states());
    }

    // Let last FOC replies finish before Phase 4 TX (avoids a systematic CRC at boundary).
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    std::cout << "Phase 4: stop (mode 0)\n";
    runFor(bus, 500, opt.rate_hz, [&](float) { sendStop(bus); });
    bus.stop();
    printStates(bus.states());
    bus.close();
    std::cout << "Done.\n";
    return 0;
}

/** Motors only reply after a control packet; probe with zero-torque FOC. */
bool waitForFeedback(ArmMotorBus& bus, int rate_hz, int timeout_ms) {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / rate_hz);
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next = clock::now();

    std::cout << "Probing feedback (zero-torque FOC, timeout "
              << timeout_ms << " ms)...\n";

    while (g_running && clock::now() < deadline) {
        sendZeroTorque(bus);
        if (bus.allFeedbackReady()) {
            std::cout << "Feedback ready for all " << kArmJointCount
                      << " joints.\n";
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

    std::cout << "ArmMotorBus low-level test\n"
              << "  hand=" << (opt.hand == Hand::Left ? "left" : "right")
              << " port=" << opt.port << " baud=" << opt.baud
              << " joints=" << kArmJointCount
              << " rate=" << opt.rate_hz << " Hz\n";

    if (opt.mode == RunMode::Stop) {
        std::cout << "Command: --stop (mode 0)\n";
        runFor(bus, 500, opt.rate_hz, [&](float) { sendStop(bus); });
        bus.stop();
        printStates(bus.states());
        bus.close();
        std::cout << "Done.\n";
        return 0;
    }

    if (opt.mode == RunMode::Clear) {
        std::cout << "Command: --clear (clearFault all joints)\n";
        // Probe once so we can see errors before clear.
        waitForFeedback(bus, opt.rate_hz, /*timeout_ms=*/2000);
        std::cout << "Before clear:\n";
        printStates(bus.states());

        for (int round = 0; round < 5 && g_running; ++round) {
            bus.clearFaults();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Refresh feedback after clear.
        waitForFeedback(bus, opt.rate_hz, /*timeout_ms=*/2000);
        std::cout << "After clear:\n";
        printStates(bus.states());
        bus.close();
        std::cout << "Done.\n";
        return 0;
    }

    if (opt.mode == RunMode::Reset) {
        std::cout << "Command: --reset (reset all joints)\n";
        waitForFeedback(bus, opt.rate_hz, /*timeout_ms=*/2000);
        std::cout << "Before reset:\n";
        printStates(bus.states());

        for (int round = 0; round < 3 && g_running; ++round) {
            bus.resetMotors();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        waitForFeedback(bus, opt.rate_hz, /*timeout_ms=*/3000);
        std::cout << "After reset:\n";
        printStates(bus.states());
        bus.close();
        std::cout << "Done.\n";
        return 0;
    }

    if (!waitForFeedback(bus, opt.rate_hz, /*timeout_ms=*/3000)) {
        std::cerr << "Abort: refuse FOC hold on default zeros "
                     "(no motor feedback yet).\n";
        sendStop(bus);
        bus.stop();
        bus.close();
        return 1;
    }

    auto q0_state = bus.states();
    std::array<float, kArmJointCount> q0{};
    for (int i = 0; i < kArmJointCount; ++i) {
        q0[i] = q0_state[i].q;
    }
    std::cout << "Start pose:\n";
    printStates(q0_state);

    if (opt.mode == RunMode::Foc) {
        std::cout << "Command: --foc hold current pose  kp=" << opt.kp
                  << " kd=" << opt.kd << " (Ctrl-C to stop)\n";
        runUntilStopped(bus, opt.rate_hz, [&]() {
            sendFocPosition(bus, q0, opt.kp, opt.kd);
        });
        shutdown(bus, opt.rate_hz);
        std::cout << "Done.\n";
        return 0;
    }

    if (opt.mode == RunMode::ZeroTorque) {
        std::cout << "Command: --zero-torque (FOC kp=kd=tau=0, Ctrl-C to stop)\n";
        runUntilStopped(bus, opt.rate_hz, [&]() { sendZeroTorque(bus); });
        shutdown(bus, opt.rate_hz);
        std::cout << "Done.\n";
        return 0;
    }

    if (opt.mode == RunMode::Home) {
        std::array<float, kArmJointCount> q_home{};
        return runCreepSequence(bus, opt, q0, q_home, "--home  creep to q=0");
    }

    if (opt.mode == RunMode::Goto) {
        return runCreepSequence(bus, opt, q0, opt.q_target,
                                "--goto  creep to configured q");
    }

    if (opt.mode == RunMode::GotoHold) {
        return runCreepSequence(bus, opt, q0, opt.q_target,
                                "--goto-hold  creep to configured q and hold",
                                /*hold_forever=*/true);
    }

    std::cerr << "Internal error: unhandled mode\n";
    bus.close();
    return 1;
}
