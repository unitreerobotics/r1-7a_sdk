<h1 align="center">r1-7a_sdk</h1>

<p align="center"><a href="README.md">简体中文</a> | <a href="README_en.md">English</a></p>

<p align="center">R1 机械臂底层控制 SDK。</p>

### 预构建环境
* 操作系统  (Ubuntu 20.04 LTS 或更高版本)
* CPU  (aarch64 和 x86_64)
* 编译器  (gcc 9.4.0)

### 环境配置

在构建或运行 SDK 之前，请确保已安装以下依赖：

- CMake (3.10 或更高版本)
- GCC (9.4.0)
- Make
- pthread

在 Ubuntu 20.04 上，可通过以下命令安装所需软件包：

```bash
apt-get update
apt-get install -y cmake g++ build-essential
```

访问串口通常需要当前用户加入 `dialout` 组（或等效组），以便访问 `/dev/ttyUSB*`。

### 构建示例

本项目提供 `build.sh` 脚本，用于选择目标架构并自动链接对应的预编译 `motor_controller` 库。

```bash
# x86_64
./build.sh x86

# aarch64
./build.sh arm64
```

该脚本会创建 `build/` 目录，使用正确的 `MOTOR_CONTROLLER_LIB_DIR` 运行 CMake，并完成构建。

如果你希望手动运行 CMake，则必须为目标架构指定 `MOTOR_CONTROLLER_LIB_DIR`：

```bash
mkdir build
cd build

# x86_64
cmake .. -DMOTOR_CONTROLLER_LIB_DIR=../thirdparty/motor_controller/lib/x86

# aarch64
cmake .. -DMOTOR_CONTROLLER_LIB_DIR=../thirdparty/motor_controller/lib/aarch64

make
```

构建产物位于 `bin/` 目录：

- `test_arm_lowlevel` — 底层 FOC / 停止 / 回零 / 目标位置命令

### 使用说明

默认串口为 `/dev/ttyUSB3`，波特率为 `6000000`。`test_arm_lowlevel` 支持可选的手部标志：`-r` / `--right`（加载 `config/joint_calib_right.txt`，未指定时默认使用）或 `-l` / `--left`（加载 `config/joint_calib_left.txt`，为右手标定镜像后的副本）。关节标定文件路径相对于当前工作目录；从 `bin/` 运行时，默认使用 `../config/...` 路径。

**保持当前位置（FOC）：**

```bash
cd bin
./test_arm_lowlevel --foc -r --port /dev/ttyUSB3
```

**运动到 URDF 零位（home）：**

```bash
./test_arm_lowlevel --home -r --kp 60
```

**运动到目标位置（goto）：**

```bash
./test_arm_lowlevel --goto -r --q 0 0 0 0 0 0 0 0 --kp 60
```

**其他命令：** `--stop`、`--zero-torque`、`--clear`、`--reset`。每次运行只能使用一个命令；添加 `-r` 或 `-l` 选择手部。长时间运行模式下，按 Ctrl-C 会发送停止指令（mode 0）并退出。

第 8 个电机（关节 7）为末端执行器电机，电机实际运动行程为 `0-5.38`，经标定缩放后映射到 URDF 坐标系下的 `0-0.044`。

### 说明

本软件包通过 vendored 的 `thirdparty/motor_controller`（IM6014）以单串口总线驱动 8 自由度 R1 机械臂。对外公开的 `q` / `dq` / `tau` 使用 URDF 坐标系；坐标转换在 `ArmMotorBus` 层完成。
