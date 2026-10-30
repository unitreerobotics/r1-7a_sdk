# r1-7a_sdk
R1 robotic arm low-level control SDK (IM6014 serial bus).

### Prebuild environment
* OS  (Ubuntu 20.04 LTS or higher)
* CPU  (aarch64 and x86_64)
* Compiler  (gcc version 9.4.0)

### Environment Setup

Before building or running the SDK, ensure the following dependencies are installed:

- CMake (version 3.10 or higher)
- GCC (version 9.4.0)
- Make
- pthread

You can install the required packages on Ubuntu 20.04 with:

```bash
apt-get update
apt-get install -y cmake g++ build-essential
```

Serial access usually requires membership in the `dialout` group (or equivalent) for `/dev/ttyUSB*`.

### Build examples

To build the examples inside this repository:

```bash
mkdir build
cd build
cmake ..
make
```

Binaries are written to `bin/`:

- `test_arm_lowlevel` — low-level FOC / stop / home / goto commands
- `calibrate_joints` — joint calibration tool (URDF ↔ motor frame)
- `demo_arm_motion` — pre-programmed joint motion demo

### Usage

Default serial port is `/dev/ttyUSB3`, baudrate `6000000`. `test_arm_lowlevel` and `demo_arm_motion` accept an optional hand flag: `-r` / `--right` (loads `config/joint_calib_right.txt`, the default when no flag is given) or `-l` / `--left` (loads `config/joint_calib_left.txt`, a dir/offset-mirrored copy of the right-hand calib). Joint calibration paths are relative to the working directory; when run from `bin/`, the default `../config/...` paths are used.

**Hold current pose (FOC):**

```bash
cd bin
./test_arm_lowlevel --foc -r --port /dev/ttyUSB3
```

**Move to URDF zero (home):**

```bash
./test_arm_lowlevel --home -r --kp 60
```

**Move to a target pose (goto):**

```bash
./test_arm_lowlevel --goto -r --q 0 0 0 0 0 0 0 --kp 60
```

**Other commands:** `--stop`, `--zero-torque`, `--clear`, `--reset`. Use exactly one command per run; add `-r` or `-l` to select the hand (defaults to right). Ctrl-C sends stop (mode 0) and exits for long-running modes.

**Run the pre-programmed motion demo:**

```bash
cd bin
./demo_arm_motion -r --port /dev/ttyUSB3
```

The demo waits for feedback from all 7 joints, enables FOC at the current pose, and then runs a fixed sequence of cosine-profile moves. Use `-r` / `-l` to select the hand (defaults to right), `--speed` and `--wrist-speed` to adjust joint velocities, `--kp` / `--kd` to tune gains, and `--rate` to change the control loop frequency (default 100 Hz). Ctrl-C stops all motors and exits.

### Notice

This package drives a 7-DOF R1 arm over one serial bus via the vendored `thirdparty/motor_controller` (IM6014). Public `q` / `dq` / `tau` use the URDF frame; conversion happens at the `ArmMotorBus` edge.
