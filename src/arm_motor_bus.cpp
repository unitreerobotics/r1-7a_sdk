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
#include <cstring>

#include "unitreeMotor/include/motor_msg_HG.h"

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

// ---------------------------------------------------------------------------
// M4010 (Unitree HG protocol) helpers for the last gripper joint.
// The calibration for the gripper joint maps URDF <-> output frame; below we
// convert output <-> rotor using the M4010 gear ratio.
// ---------------------------------------------------------------------------
constexpr float kM4010GearRatio = 25.0f;
constexpr float kM4010TwoPi = 6.283185307179586f;

constexpr float kM4010TauFactor = 256.0f * 10.0f;
constexpr float kM4010TauInvFactor = 1.0f / kM4010TauFactor;
constexpr float kM4010SpeedFactor = 256.0f / kM4010TwoPi;
constexpr float kM4010SpeedInvFactor = kM4010TwoPi / 256.0f;
constexpr float kM4010PosFactor = 32768.0f / kM4010TwoPi;
constexpr float kM4010PosInvFactor = kM4010TwoPi / 32768.0f;
constexpr float kM4010KpFactor = 1280.0f * 10.0f;
constexpr float kM4010KdFactor = 1280.0f * 10.0f;

float clampFloat(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

std::vector<uint8_t> buildM4010ControlPacket(
    uint8_t motor_id, uint8_t mode, bool timeout,
    float tau, float dq, float q, float kp, float kd) {
    alignas(uint32_t) ControlData_hg pkt{};
    pkt.head[0] = 0xFE;
    pkt.head[1] = 0xEE;
    pkt.id = motor_id & 0x0F;
    pkt.status = mode & 0x07;
    pkt.timeout = timeout ? 1 : 0;
    pkt.res = 0;

    tau = clampFloat(tau, -15.0f, 15.0f);
    pkt.tor_des = static_cast<int16_t>(tau * kM4010TauFactor);
    pkt.spd_des = static_cast<int32_t>(dq * kM4010SpeedFactor);
    pkt.pos_des = static_cast<int32_t>(q * kM4010PosFactor);
    pkt.k_pos = static_cast<uint16_t>(kp * kM4010KpFactor);
    pkt.k_spd = static_cast<uint16_t>(kd * kM4010KdFactor);

    pkt.CRC32 = crc32_core(reinterpret_cast<uint32_t*>(&pkt),
                           (sizeof(pkt) >> 2) - 1);

    std::vector<uint8_t> out(sizeof(pkt));
    std::memcpy(out.data(), &pkt, sizeof(pkt));
    return out;
}

// Clear fault, mirrors IM6014 buildClearPacket: status=6, tor_des=-256.
std::vector<uint8_t> buildM4010ClearPacket(uint8_t motor_id) {
    alignas(uint32_t) ControlData_hg pkt{};
    pkt.head[0] = 0xFE;
    pkt.head[1] = 0xEE;
    pkt.id = motor_id & 0x0F;
    pkt.status = 6;
    pkt.timeout = 0;
    pkt.res = 0;
    pkt.tor_des = -256;
    pkt.spd_des = 0;
    pkt.pos_des = 0;
    pkt.k_pos = 0;
    pkt.k_spd = 0;
    pkt.CRC32 = crc32_core(reinterpret_cast<uint32_t*>(&pkt),
                           (sizeof(pkt) >> 2) - 1);
    std::vector<uint8_t> out(sizeof(pkt));
    std::memcpy(out.data(), &pkt, sizeof(pkt));
    return out;
}

// Reset motor, mirrors IM6014 buildResetPacket: status=7, all zeros.
std::vector<uint8_t> buildM4010ResetPacket(uint8_t motor_id) {
    alignas(uint32_t) ControlData_hg pkt{};
    pkt.head[0] = 0xFE;
    pkt.head[1] = 0xEE;
    pkt.id = motor_id & 0x0F;
    pkt.status = 7;
    pkt.timeout = 0;
    pkt.res = 0;
    pkt.tor_des = 0;
    pkt.spd_des = 0;
    pkt.pos_des = 0;
    pkt.k_pos = 0;
    pkt.k_spd = 0;
    pkt.CRC32 = crc32_core(reinterpret_cast<uint32_t*>(&pkt),
                           (sizeof(pkt) >> 2) - 1);
    std::vector<uint8_t> out(sizeof(pkt));
    std::memcpy(out.data(), &pkt, sizeof(pkt));
    return out;
}

struct M4010Feedback {
    uint8_t motor_id = 0;
    uint8_t mode = 0;
    uint8_t timeout = 0;
    int temp = 0;
    float tau = 0.f;
    float dq = 0.f;
    float q = 0.f;
    uint32_t error_code = 0;
    bool correct = false;
};

M4010Feedback parseM4010FeedbackPacket(const std::vector<uint8_t>& data) {
    M4010Feedback fb;
    if (data.size() != sizeof(MotorData_hg)) {
        return fb;
    }
    alignas(uint32_t) MotorData_hg raw{};
    std::memcpy(&raw, data.data(), sizeof(raw));
    if (raw.head[0] != 0xFC || raw.head[1] != 0xEE) {
        return fb;
    }

    uint32_t crc_words[(sizeof(MotorData_hg) - 6) >> 2];
    std::memcpy(crc_words,
                reinterpret_cast<const uint8_t*>(&raw) + 2,
                sizeof(crc_words));
    const uint32_t expected_crc = crc32_core(crc_words, sizeof(crc_words) >> 2);
    if (raw.CRC32 != expected_crc) {
        return fb;
    }

    fb.motor_id = raw.id;
    fb.mode = raw.status;
    fb.timeout = raw.timeout;
    fb.temp = static_cast<int>(raw.temp1);
    fb.tau = static_cast<float>(raw.torque) * kM4010TauInvFactor;
    fb.dq = static_cast<float>(raw.speed) * kM4010SpeedInvFactor;
    fb.q = static_cast<float>(raw.pos) * kM4010PosInvFactor;
    fb.error_code = raw.MError;
    fb.correct = true;
    return fb;
}

}  // namespace

struct ArmMotorBus::Impl {
    SerialPort serial;
    MotorProtocol protocol;
    std::array<uint8_t, kArmJointCount> motor_ids{};
    std::array<JointCalibration, kArmJointCount> calib{};
    mutable std::mutex fb_mutex;
    std::array<ArmJointState, kArmJointCount> latest_motor{};  // motor frame (output frame for gripper)
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
        if (data.size() < 3) {
            return;
        }
        const uint8_t raw_id = data[2] & 0x0F;

        // Gripper joint uses the M4010 / Unitree HG feedback format.
        // latest_motor stores the output-side frame; states() converts to URDF.
        if (raw_id == motor_ids[kGripperJointIndex]) {
            auto fb = parseM4010FeedbackPacket(data);
            if (!fb.correct) {
                return;
            }

            std::lock_guard<std::mutex> lock(fb_mutex);
            latest_motor[kGripperJointIndex].q = fb.q / kM4010GearRatio;
            latest_motor[kGripperJointIndex].dq = fb.dq / kM4010GearRatio;
            latest_motor[kGripperJointIndex].tau_est = fb.tau * kM4010GearRatio;
            latest_motor[kGripperJointIndex].mode = fb.mode;
            latest_motor[kGripperJointIndex].timeout = fb.timeout;
            latest_motor[kGripperJointIndex].error_code = fb.error_code;
            fb_ready[kGripperJointIndex] = true;
            return;
        }

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
                if (i == kGripperJointIndex) {
                    auto pkt = buildM4010ControlPacket(
                        motor_ids[i], /*mode=*/0, /*timeout=*/false,
                        0.f, 0.f, 0.f, 0.f, 0.f);
                    serial.write(pkt);
                } else {
                    auto pkt = protocol.buildControlPacket(
                        motor_ids[i], /*mode=*/0, /*timeout=*/false,
                        0.f, 0.f, 0.f, 0.f, 0.f);
                    serial.write(pkt);
                }
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
            if (i == kGripperJointIndex) {
                // M4010 gripper: calibration maps URDF <-> output frame.
                if (c.timed_out) {
                    packet = buildM4010ControlPacket(
                        impl_->motor_ids[i], /*mode=*/1, /*timeout=*/true,
                        0.f, 0.f, 0.f, 0.f, 0.f);
                } else {
                    const uint8_t mode = (c.mode == 0) ? 0 : 1;
                    const float output_q = urdfToMotorAngle(c.q, cal);
                    const float output_dq = urdfToMotorVec(c.dq, cal);
                    const float output_tau = urdfToMotorVec(c.tau, cal);
                    const float rotor_q = output_q * kM4010GearRatio;
                    const float rotor_dq = output_dq * kM4010GearRatio;
                    const float rotor_tau = output_tau / kM4010GearRatio;
                    const float rotor_kp = c.kp / (kM4010GearRatio * kM4010GearRatio);
                    const float rotor_kd = c.kd / (kM4010GearRatio * kM4010GearRatio);
                    packet = buildM4010ControlPacket(
                        impl_->motor_ids[i], mode, /*timeout=*/(mode == 1),
                        rotor_tau, rotor_dq, rotor_q, rotor_kp, rotor_kd);
                }
            } else {
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
            // M4010 gripper uses the HG-format clear packet (status=6).
            auto pkt = (i == kGripperJointIndex)
                           ? buildM4010ClearPacket(impl_->motor_ids[i])
                           : impl_->protocol.buildClearPacket(impl_->motor_ids[i]);
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
            // M4010 gripper uses the HG-format reset packet (status=7).
            auto pkt = (i == kGripperJointIndex)
                           ? buildM4010ResetPacket(impl_->motor_ids[i])
                           : impl_->protocol.buildResetPacket(impl_->motor_ids[i]);
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
