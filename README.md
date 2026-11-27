<h1 align="center">r1-7a_sdk</h1>

<p align="center"><a href="README.md">简体中文</a> | <a href="README_en.md">English</a></p>

<p align="center">R1 机械臂底层控制 SDK。</p>

### 预构建环境
* 操作系统  (必须使用 Ubuntu 24.04 LTS)
* CPU  (aarch64 和 x86_64)

### 环境配置

在构建或运行 SDK 之前，请确保已安装以下依赖：

- CMake (3.28 或更高版本)
- Make
- pthread

在 Ubuntu 24.04 上，可通过以下命令安装所需软件包：

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

算法示例默认关闭。添加 `-with-algo`（也支持 `--with-algo`）后，`bin/` 中还会生成 `gravity_compensation`（重力补偿）。

```bash
sudo apt-get install -y libeigen3-dev libboost-filesystem-dev liburdfdom-dev
./build.sh x86 -with-algo
# 在 aarch64 主机上构建
./build.sh arm64 -with-algo
```

算法构建从 `algo/pinocchio/lib/<架构>/` 链接预编译的 Pinocchio 库。手动配置 CMake 时，在下方对应架构的命令中追加 `-DBUILD_WITH_ALGO=ON`。CMake 默认按编译目标架构选择 Pinocchio，也可以用 `-DARM_TARGET_PLATFORM=x86_64` 或 `-DARM_TARGET_PLATFORM=aarch64` 显式指定。跨架构构建还需要为 CMake 提供对应的工具链。

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
- `demo_arm_motion` — 预设关节运动演示

### 使用说明

默认串口为 `/dev/ttyUSB3`，波特率为 `6000000`。`test_arm_lowlevel` 和 `demo_arm_motion` 支持可选的手部标志：`-r` / `--right`（加载 `config/joint_calib_right.txt`，未指定时默认使用）或 `-l` / `--left`（加载 `config/joint_calib_left.txt`，为右手标定镜像后的副本）。关节标定文件路径相对于当前工作目录；从 `bin/` 运行时，默认使用 `../config/...` 路径。

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
./test_arm_lowlevel --goto -r --q 0 0 0 0 0 0 0 --kp 60
```

**其他命令：** `--stop`、`--zero-torque`、`--clear`、`--reset`。每次运行只能使用一个命令；添加 `-r` 或 `-l` 选择手部。长时间运行模式下，按 Ctrl-C 会发送停止指令（mode 0）并退出。

**运行预设运动演示：**

```bash
cd bin
./demo_arm_motion -r --port /dev/ttyUSB3
```

演示程序会等待 7 个关节全部反馈到位，然后在当前位置启用 FOC，并执行一段固定的余弦轨迹运动。使用 `-r` / `-l` 选择手部，`--speed` 和 `--wrist-speed` 调整关节速度，`--kp` / `--kd` 调整增益，`--rate` 调整控制频率（默认 100 Hz）。按 Ctrl-C 可停止所有电机并退出。

**运行重力补偿（需使用 `-with-algo` 构建）：**

```bash
cd bin
./gravity_compensation -r  # 右臂
./gravity_compensation -l  # 左臂
```

`-r` / `-l` 分别加载对应手臂的标定文件和 URDF；未指定时默认右臂。启动后等待 7 个关节反馈，再进入重力补偿模式；按 Ctrl-C 发送停止指令并退出。操作前请确保机械臂有支撑，避免突然下落。

**动作记录与回放（需使用 `-with-algo` 构建）：**

```bash
cd bin
./roboplan_record -r  # 右臂
./roboplan_record -l  # 左臂
```

程序启动后，手臂先进入零重力模式：

1. 按回车，开始记录动作，可用手移动机械臂。
2. 再次按回车，停止记录。手臂缓慢回到记录轨迹的起始位置，然后自动回放动作。
3. 回放结束后，手臂自动进入零重力模式。按回车进入下一轮流程，或输入 `q` 并按回车退出程序。
4. 返回步骤 1，重复记录与回放。

注意：当前程序在步骤 3 按回车后，仍需在步骤 1 再按一次回车，才会开始下一次记录。每次记录的轨迹默认保存到 `csv/recorded_traj.csv`，可通过 `-o PATH` 指定输出路径。操作前请确保机械臂有支撑，避免突然下落。

### 说明

本软件包通过 vendored 的 `thirdparty/motor_controller`（IM6014）以单串口总线驱动 7 自由度 R1 机械臂。对外公开的 `q` / `dq` / `tau` 使用 URDF 坐标系；坐标转换在 `ArmMotorBus` 层完成。
