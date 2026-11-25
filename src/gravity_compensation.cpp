// Zero-gravity (gravity compensation) demo for the R1 7-DoF arm.
//
// At every control tick the current joint positions are read back from the
// motors, fed into Pinocchio's inverse dynamics (computeGeneralizedGravity)
// together with the URDF model, and the resulting per-joint gravity torque
// is sent back out as a pure feed-forward torque command (kp = kd = 0, so
// the arm should feel weightless and can be moved freely by hand while the
// motors still actively cancel gravity in any pose).

#include "arm_motor_bus.h"

#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {

using JointArray = std::array<float, kArmJointCount>;
using Clock = std::chrono::steady_clock;

std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

constexpr const char* kRightUrdfPath =
    "../urdf/R1_Robotic_Arm_R_V1.2-1023/R1_Robotic_Arm_V1.2/R1_Robotic_Arm/urdf/"
    "R1_Robotic_Arm.urdf";
constexpr const char* kLeftUrdfPath =
    "../urdf/R1_Robotic_Arm_L_V1.0-1027/R1_Robotic_Arm_L_V1.0-1027/meshes/"
    "R1_Robotic_Arm_L_V1.0-1027.urdf";

enum class Hand { None, Right, Left };

struct Options {
    std::string port = "/dev/ttyUSB3";
    int baud = kArmDefaultBaudrate;
    std::string urdf_path;
    float tau_limit = 15.f;   // clamp |tau_ff| per joint (Nm), safety margin
    float kd = 0.3f;          // small damping to curb drift/oscillation
    int rate_hz = 200;
    Hand hand = Hand::None;
};

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [-r|-l] [options]\n"
        << "Gravity compensation (\"zero gravity\") mode: holds zero net torque\n"
        << "at the current pose using Pinocchio RNEA; the arm can be moved by hand.\n"
        << "Hand (optional, default right):\n"
        << "  -r, --right       right arm (calib " << kRightJointCalibPath << ")\n"
        << "                    default URDF: " << kRightUrdfPath << "\n"
        << "  -l, --left        left arm (calib " << kLeftJointCalibPath << ")\n"
        << "                    default URDF: " << kLeftUrdfPath << "\n"
        << "Options:\n"
        << "  --port PATH       serial device (default /dev/ttyUSB3)\n"
        << "  --baud N          baudrate (default " << kArmDefaultBaudrate << ")\n"
        << "  --urdf PATH       override the selected arm's default URDF\n"
        << "  --tau-limit F     clamp feed-forward torque per joint, Nm (default 15)\n"
        << "  --kd F            light damping Nm/(rad/s) (default 0.3)\n"
        << "  --rate N          control rate Hz (default 200)\n"
        << "Ctrl-C sends stop (mode 0) and exits.\n";
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
        } else if (std::strcmp(argv[i], "--tau-limit") == 0 && has_val) {
            opt.tau_limit = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--kd") == 0 && has_val) {
            opt.kd = std::stof(argv[++i]);
        } else if (std::strcmp(argv[i], "--rate") == 0 && has_val) {
            opt.rate_hz = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown or incomplete arg: " << argv[i] << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    if (opt.rate_hz <= 0) {
        opt.rate_hz = 200;
    }
    if (opt.hand == Hand::None) {
        opt.hand = Hand::Right;
    }
    if (opt.urdf_path.empty()) {
        opt.urdf_path = (opt.hand == Hand::Left) ? kLeftUrdfPath : kRightUrdfPath;
    }
    opt.tau_limit = std::max(0.f, opt.tau_limit);
    opt.kd = std::max(0.f, opt.kd);
    return true;
}

void printStates(const std::array<ArmJointState, kArmJointCount>& s,
                  const Eigen::VectorXd& tau_ff) {
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  j" << i << " q=" << s[i].q
                  << " dq=" << s[i].dq
                  << " tau_meas=" << s[i].tau_est
                  << " tau_ff=" << tau_ff[i]
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

void sendZeroTorque(ArmMotorBus& bus) {
    std::array<ArmJointCmd, kArmJointCount> cmds{};
    for (int i = 0; i < kArmJointCount; ++i) {
        cmds[i].mode = 1;
        cmds[i].timed_out = false;
    }
    bus.step(cmds);
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
    bus.close();
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

    // ----- Build the Pinocchio model from URDF -----
    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(opt.urdf_path, model);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load URDF '" << opt.urdf_path << "': " << e.what()
                  << std::endl;
        return 1;
    }
    if (model.nq != kArmJointCount || model.nv != kArmJointCount) {
        std::cerr << "Unexpected model size: nq=" << model.nq << " nv=" << model.nv
                  << " (expected " << kArmJointCount << "). Check the URDF path.\n";
        return 1;
    }
    pinocchio::Data data(model);
    std::cout << "Loaded Pinocchio model '" << model.name << "' from " << opt.urdf_path
              << " (nq=" << model.nq << ")\n";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const std::string calib_path =
        (opt.hand == Hand::Left) ? kLeftJointCalibPath : kRightJointCalibPath;

    ArmMotorBus bus;
    if (!bus.open(opt.port, opt.baud, calib_path)) {
        return 1;
    }

    std::cout << "R1 arm gravity compensation (\"zero gravity\")\n"
              << "  hand=" << (opt.hand == Hand::Left ? "left" : "right")
              << " port=" << opt.port << " tau_limit=" << opt.tau_limit
              << " kd=" << opt.kd << " rate=" << opt.rate_hz << " Hz\n";

    if (!waitForFeedback(bus, opt.rate_hz, 3000)) {
        std::cerr << "Abort: no motor feedback.\n";
        shutdown(bus, opt.rate_hz);
        return 1;
    }

    const auto period = std::chrono::microseconds(1000000 / opt.rate_hz);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(kArmJointCount);
    Eigen::VectorXd tau_ff = Eigen::VectorXd::Zero(kArmJointCount);

    std::cout << "Entering zero-gravity loop. Ctrl-C to stop.\n";
    auto next = Clock::now();
    auto last_print = Clock::now();
    while (g_running) {
        const auto st = bus.states();

        for (int i = 0; i < kArmJointCount; ++i) {
            q[i] = static_cast<double>(st[i].q);
        }

        // Static gravity torque g(q) at zero velocity/acceleration (RNEA with
        // v = a = 0): the feed-forward torque needed to hold the pose still.
        const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(model, data, q);
        tau_ff = g;
        for (int i = 0; i < kArmJointCount; ++i) {
            double tau = tau_ff[i];
            if (tau > opt.tau_limit) tau = opt.tau_limit;
            if (tau < -opt.tau_limit) tau = -opt.tau_limit;
            tau_ff[i] = tau;
        }

        std::array<ArmJointCmd, kArmJointCount> cmds{};
        for (int i = 0; i < kArmJointCount; ++i) {
            cmds[i].mode = 1;
            cmds[i].timed_out = false;
            cmds[i].q = st[i].q;        // kp = 0, so q is unused for torque but kept for telemetry
            cmds[i].dq = 0.f;
            cmds[i].tau = static_cast<float>(tau_ff[i]);
            cmds[i].kp = 0.f;           // pure torque mode: no position stiffness
            cmds[i].kd = opt.kd;        // light damping to curb free-swing oscillation
        }
        bus.step(cmds);

        if (Clock::now() - last_print >= std::chrono::milliseconds(500)) {
            printStates(st, tau_ff);
            last_print = Clock::now();
        }

        next += period;
        std::this_thread::sleep_until(next);
    }

    shutdown(bus, opt.rate_hz);
    std::cout << "Done.\n";
    return 0;
}
