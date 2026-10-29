#include "arm_motor_bus.h"

#include "SerialPort.h"
#include "MotorProtocol.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

float safeDir(float dir) {
    return (std::fabs(dir) < 1e-6f) ? 1.f : dir;
}

float safeScale(float scale) {
    return (std::fabs(scale) < 1e-6f) ? 1.f : std::fabs(scale);
}

/** Signed axis gain: dir * scale. */
float axisGain(const JointCalibration& cal) {
    return safeDir(cal.dir) * safeScale(cal.scale);
}

float urdfToMotorAngle(float urdf_q, const JointCalibration& cal) {
    return (urdf_q - cal.offset) / axisGain(cal);
}

float urdfToMotorVec(float urdf_v, const JointCalibration& cal) {
    return urdf_v / axisGain(cal);
}

float motorToUrdfAngle(float motor_q, const JointCalibration& cal) {
    return axisGain(cal) * motor_q + cal.offset;
}

float motorToUrdfVec(float motor_v, const JointCalibration& cal) {
    return axisGain(cal) * motor_v;
}

}  // namespace

struct ArmMotorBus::Impl {
    SerialPort serial;
    MotorProtocol protocol;
    std::array<uint8_t, kArmJointCount> motor_ids{};
    std::array<JointCalibration, kArmJointCount> calib{};
    mutable std::mutex fb_mutex;
    std::array<ArmJointState, kArmJointCount> latest_motor{};  // raw motor frame
    std::array<bool, kArmJointCount> fb_ready{};
    bool opened = false;
    std::string port;
    int baudrate = kArmDefaultBaudrate;

    Impl() {
        for (int i = 0; i < kArmJointCount; ++i) {
            motor_ids[i] = static_cast<uint8_t>(i);
            calib[i] = JointCalibration{1.f, 1.f, 0.f};
        }
    }

    void onPacket(const std::vector<uint8_t>& data) {
        auto fb = protocol.parseFeedbackPacket(data);
        if (!fb) {
            return;
        }

        int joint = -1;
        for (int i = 0; i < kArmJointCount; ++i) {
            if (motor_ids[i] == fb->motor_id) {
                joint = i;
                break;
            }
        }
        if (joint < 0) {
            return;
        }

        std::lock_guard<std::mutex> lock(fb_mutex);
        latest_motor[joint].q = fb->position;
        latest_motor[joint].dq = fb->speed;
        latest_motor[joint].tau_est = fb->torque;
        latest_motor[joint].mode = fb->mode;
        latest_motor[joint].timeout = fb->timeout;
        latest_motor[joint].error_code = fb->error_code;
        fb_ready[joint] = true;
    }

    void softStopAll() {
        if (!opened) {
            return;
        }
        for (int i = 0; i < kArmJointCount; ++i) {
            try {
                auto pkt = protocol.buildControlPacket(
                    motor_ids[i], /*mode=*/0, /*timeout=*/false,
                    0.f, 0.f, 0.f, 0.f, 0.f);
                serial.write(pkt);
            } catch (...) {
            }
        }
    }
};

ArmMotorBus::ArmMotorBus() : impl_(std::make_unique<Impl>()) {}

ArmMotorBus::~ArmMotorBus() {
    close();
}

bool ArmMotorBus::open(const std::string& port,
                       int baudrate,
                       const std::string& calib_path) {
    close();

    if (!loadJointCalibration(calib_path)) {
        std::cerr << "[ArmMotorBus] loadJointCalibration failed: " << calib_path
                  << " — exiting" << std::endl;
        std::exit(1);
    }

    impl_->port = port;
    impl_->baudrate = baudrate;

    if (!impl_->serial.open(port, baudrate, 0.05)) {
        std::cerr << "[ArmMotorBus] open failed: " << port << std::endl;
        return false;
    }

    impl_->serial.startAsyncRead([this](const std::vector<uint8_t>& packet) {
        impl_->onPacket(packet);
    });
    {
        std::lock_guard<std::mutex> lock(impl_->fb_mutex);
        impl_->latest_motor = {};
        impl_->fb_ready = {};
    }
    impl_->opened = true;
    std::cout << "[ArmMotorBus] opened " << port << " @ " << baudrate
              << " (calib=" << calib_path << ")" << std::endl;
    return true;
}

bool ArmMotorBus::loadJointCalibration(const std::string& path) {
    if (!impl_) {
        return false;
    }

    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "[ArmMotorBus] cannot open calib file: " << path << std::endl;
        return false;
    }

    std::array<JointCalibration, kArmJointCount> loaded{};
    std::array<bool, kArmJointCount> seen{};
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream ss(line);
        int joint = -1;
        float dir = 1.f;
        float scale = 1.f;
        float offset = 0.f;
        float a = 0.f;
        float b = 0.f;
        if (!(ss >> joint >> dir >> a)) {
            std::cerr << "[ArmMotorBus] bad calib line: " << line << std::endl;
            return false;
        }
        if (ss >> b) {
            // joint dir scale offset
            scale = a;
            offset = b;
        } else {
            // legacy: joint dir offset
            offset = a;
            scale = 1.f;
        }
        if (joint < 0 || joint >= kArmJointCount) {
            std::cerr << "[ArmMotorBus] joint index out of range: " << joint
                      << std::endl;
            return false;
        }
        if (std::fabs(dir) < 1e-6f) {
            std::cerr << "[ArmMotorBus] invalid dir for joint " << joint
                      << std::endl;
            return false;
        }
        if (std::fabs(scale) < 1e-6f) {
            std::cerr << "[ArmMotorBus] invalid scale for joint " << joint
                      << std::endl;
            return false;
        }
        loaded[joint] = JointCalibration{safeDir(dir), safeScale(scale), offset};
        seen[joint] = true;
    }

    for (int i = 0; i < kArmJointCount; ++i) {
        if (!seen[i]) {
            std::cerr << "[ArmMotorBus] missing joint " << i << " in " << path
                      << std::endl;
            return false;
        }
    }

    impl_->calib = loaded;
    std::cout << "[ArmMotorBus] loaded calib from " << path << ":\n";
    for (int i = 0; i < kArmJointCount; ++i) {
        std::cout << "  j" << i << " dir=" << impl_->calib[i].dir
                  << " scale=" << impl_->calib[i].scale
                  << " offset=" << impl_->calib[i].offset << "\n";
    }
    return true;
}

void ArmMotorBus::close() {
    if (!impl_ || !impl_->opened) {
        return;
    }
    impl_->softStopAll();
    impl_->serial.stopAsyncRead();
    impl_->serial.close();
    impl_->opened = false;
}

bool ArmMotorBus::isOpen() const {
    return impl_ && impl_->opened;
}

void ArmMotorBus::setMotorId(int joint, uint8_t motor_id) {
    if (!impl_ || joint < 0 || joint >= kArmJointCount) {
        return;
    }
    impl_->motor_ids[joint] = motor_id;
}

void ArmMotorBus::setJointCalibration(int joint, float dir, float offset) {
    setJointCalibration(joint, JointCalibration{dir, 1.f, offset});
}

void ArmMotorBus::setJointCalibration(int joint, float dir, float scale, float offset) {
    setJointCalibration(joint, JointCalibration{dir, scale, offset});
}

void ArmMotorBus::setJointCalibration(int joint, const JointCalibration& cal) {
    if (!impl_ || joint < 0 || joint >= kArmJointCount) {
        return;
    }
    impl_->calib[joint] = cal;
    impl_->calib[joint].dir = safeDir(cal.dir);
    impl_->calib[joint].scale = safeScale(cal.scale);
}

JointCalibration ArmMotorBus::jointCalibration(int joint) const {
    if (!impl_ || joint < 0 || joint >= kArmJointCount) {
        return {};
    }
    return impl_->calib[joint];
}

const std::array<JointCalibration, kArmJointCount>& ArmMotorBus::jointCalibrations() const {
    return impl_->calib;
}

void ArmMotorBus::step(const std::array<ArmJointCmd, kArmJointCount>& cmds) {
    if (!isOpen()) {
        return;
    }

    for (int i = 0; i < kArmJointCount; ++i) {
        const auto& c = cmds[i];
        const auto& cal = impl_->calib[i];
        try {
            std::vector<uint8_t> packet;
            if (c.timed_out) {
                packet = impl_->protocol.buildControlPacket(
                    impl_->motor_ids[i], /*mode=*/1, /*timeout=*/true,
                    0.f, 0.f, 0.f, 0.f, 0.f);
            } else {
                const uint8_t mode = (c.mode == 0) ? 0 : 1;
                const float motor_q = urdfToMotorAngle(c.q, cal);
                const float motor_dq = urdfToMotorVec(c.dq, cal);
                const float motor_tau = urdfToMotorVec(c.tau, cal);
                packet = impl_->protocol.buildControlPacket(
                    impl_->motor_ids[i], mode, /*timeout=*/(mode == 1),
                    motor_tau, motor_dq, motor_q, c.kp, c.kd);
            }
            impl_->serial.write(packet);
            // Half-duplex RS485: small gap so replies don't collide with next TX.
            if (i + 1 < kArmJointCount + 1) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        } catch (const std::exception& e) {
            std::cerr << "[ArmMotorBus] write joint " << i
                      << " failed: " << e.what() << std::endl;
        }
    }
}

void ArmMotorBus::stop() {
    if (!impl_) {
        return;
    }
    impl_->softStopAll();
}

void ArmMotorBus::clearFaults() {
    if (!isOpen()) {
        return;
    }
    for (int i = 0; i < kArmJointCount; ++i) {
        try {
            auto pkt = impl_->protocol.buildClearPacket(impl_->motor_ids[i]);
            impl_->serial.write(pkt);
        } catch (const std::exception& e) {
            std::cerr << "[ArmMotorBus] clearFault joint " << i
                      << " failed: " << e.what() << std::endl;
        }
    }
}

void ArmMotorBus::resetMotors() {
    if (!isOpen()) {
        return;
    }
    for (int i = 0; i < kArmJointCount; ++i) {
        try {
            auto pkt = impl_->protocol.buildResetPacket(impl_->motor_ids[i]);
            impl_->serial.write(pkt);
        } catch (const std::exception& e) {
            std::cerr << "[ArmMotorBus] reset joint " << i
                      << " failed: " << e.what() << std::endl;
        }
    }
}

std::array<ArmJointState, kArmJointCount> ArmMotorBus::states() const {
    std::lock_guard<std::mutex> lock(impl_->fb_mutex);
    std::array<ArmJointState, kArmJointCount> out{};
    for (int i = 0; i < kArmJointCount; ++i) {
        const auto& m = impl_->latest_motor[i];
        const auto& cal = impl_->calib[i];
        out[i].q = motorToUrdfAngle(m.q, cal);
        out[i].dq = motorToUrdfVec(m.dq, cal);
        out[i].tau_est = motorToUrdfVec(m.tau_est, cal);
        out[i].mode = m.mode;
        out[i].timeout = m.timeout;
        out[i].error_code = m.error_code;
    }
    return out;
}

std::array<ArmJointState, kArmJointCount> ArmMotorBus::motorStates() const {
    std::lock_guard<std::mutex> lock(impl_->fb_mutex);
    return impl_->latest_motor;
}

std::array<bool, kArmJointCount> ArmMotorBus::feedbackReady() const {
    std::lock_guard<std::mutex> lock(impl_->fb_mutex);
    return impl_->fb_ready;
}

bool ArmMotorBus::allFeedbackReady() const {
    std::lock_guard<std::mutex> lock(impl_->fb_mutex);
    for (int i = 0; i < kArmJointCount; ++i) {
        if (!impl_->fb_ready[i]) {
            return false;
        }
    }
    return true;
}
