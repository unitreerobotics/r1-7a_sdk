#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

constexpr int kArmJointCount = 7;
constexpr int kArmDefaultBaudrate = 6000000;
constexpr const char* kDefaultJointCalibPath = "../config/joint_calib_right.txt";
constexpr const char* kRightJointCalibPath = "../config/joint_calib_right.txt";
constexpr const char* kLeftJointCalibPath = "../config/joint_calib_left.txt";

/**
 * URDF joint frame <-> motor frame:
 *   urdf_angle = dir * scale * motor_angle + offset
 *   motor_angle = (urdf_angle - offset) / (dir * scale)
 * dir is typically ±1; scale > 0 maps motor span onto URDF span (see joint limits);
 * dq/tau use the same dir*scale factor (no offset).
 */
struct JointCalibration {
    float dir = 1.f;
    float scale = 1.f;   // >0
    float offset = 0.f;  // rad, URDF value when motor_angle = 0
};

/** Joint-side command in URDF frame (same units as DDS / URDF). */
struct ArmJointCmd {
    float q = 0.f;       // rad (URDF)
    float dq = 0.f;      // rad/s (URDF)
    float tau = 0.f;     // Nm (URDF)
    float kp = 0.f;      // Nm/rad
    float kd = 0.f;      // Nm/(rad/s)
    uint8_t mode = 1;    // 0: stop, 1: FOC
    bool timed_out = true;
};

/** Joint-side feedback in URDF frame. */
struct ArmJointState {
    float q = 0.f;
    float dq = 0.f;
    float tau_est = 0.f;
    uint8_t mode = 0;       // 0: stop, 1: FOC (from motor feedback)
    uint8_t timeout = 0;    // motor timeout flag
    uint32_t error_code = 0;
};

/**
 * Multi-joint IM6014 bus: one serial port, motors addressed by ID.
 * Default mapping: joint[i] <-> motor_id = i.
 * Public q/dq/tau are URDF-frame; converted with JointCalibration at the bus edge.
 */
class ArmMotorBus {
public:
    ArmMotorBus();
    ~ArmMotorBus();

    ArmMotorBus(const ArmMotorBus&) = delete;
    ArmMotorBus& operator=(const ArmMotorBus&) = delete;

    /**
     * Open serial and load joint calib from file (default config/joint_calib.txt).
     * On calib load failure: prints error and calls std::exit(1).
     */
    bool open(const std::string& port,
              int baudrate = kArmDefaultBaudrate,
              const std::string& calib_path = kDefaultJointCalibPath);
    void close();
    bool isOpen() const;

    void setMotorId(int joint, uint8_t motor_id);

    void setJointCalibration(int joint, float dir, float offset);
    void setJointCalibration(int joint, float dir, float scale, float offset);
    void setJointCalibration(int joint, const JointCalibration& cal);
    JointCalibration jointCalibration(int joint) const;
    const std::array<JointCalibration, kArmJointCount>& jointCalibrations() const;

    /**
     * Load dir/scale/offset for all joints from a calib file
     * (format: "joint dir scale offset" per line; legacy "joint dir offset" => scale=1).
     * Returns false if the file is missing or any joint is incomplete.
     */
    bool loadJointCalibration(const std::string& path = kDefaultJointCalibPath);

    /** Send commands for all joints (URDF frame); feedback is updated asynchronously. */
    void step(const std::array<ArmJointCmd, kArmJointCount>& cmds);

    /** Mode 0 (stop) on every joint. */
    void stop();

    /** Send clear-fault packet to every joint. */
    void clearFaults();

    /** Send reset packet to every joint. */
    void resetMotors();

    /** Latest feedback snapshot in URDF frame. */
    std::array<ArmJointState, kArmJointCount> states() const;

    /** Latest feedback snapshot in motor frame (no dir/scale/offset). */
    std::array<ArmJointState, kArmJointCount> motorStates() const;

    /** True once a valid feedback packet has been parsed for that joint. */
    std::array<bool, kArmJointCount> feedbackReady() const;

    /** True if every joint has received at least one valid feedback. */
    bool allFeedbackReady() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
