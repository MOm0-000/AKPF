# AKPF

AKPF 是一个基于 ArduPilot 的无人机仿真与避障研究工程。当前仓库聚焦于从基础仿真链路到室内场景协议的逐层落地，为后续风险势场、强化学习避障和安全屏蔽实验提供可复现工程基础。

## 当前技术栈

```text
Ubuntu 22.04 / WSL2
ROS2 Humble
Gazebo Harmonic / Gazebo Sim 8
ArduPilot SITL / ArduCopter
MAVROS2
```

当前项目选择 ArduPilot 路线，不使用 PX4。

## 已完成层级

### L0：ArduPilot 基线链路

完成 Gazebo、ArduPilot SITL、MAVROS2 与基础状态机链路复现。

参考文档：

```text
L0_ArduPilot基线复现记录.md
```

### L1：速度闭环控制

完成 MAVROS 速度控制接口确认，并实现最小速度闭环状态机：

```text
等待 FCU -> GUIDED -> 解锁 -> 起飞 -> 前后左右上下速度测试 -> 悬停 -> 降落
```

核心文件：

```text
src/l1_velocity_control
config/mavros_l1_pluginlists.yaml
scripts/start_mavros_l1.sh
scripts/run_l1_velocity.sh
```

参考文档：

```text
L1_ArduPilot速度闭环复现教程.md
L1_ArduPilot速度闭环验收记录.md
```

### L2：室内仿真场景与实验协议

完成 5 个基础室内 Gazebo 场景和统一实验协议：

```text
S1_single_front_obstacle
S2_narrow_gate
S3_corridor
S4_table_or_low_obstacle
S5_corner
```

核心文件：

```text
worlds/l2
protocols/l2_scenarios.yaml
scripts/start_l2_world.sh
scripts/check_l2_worlds.sh
```

参考文档：

```text
L2_ArduPilot室内场景复现教程.md
L2_室内仿真场景与实验协议.md
```

## 目录结构

```text
config/      MAVROS 配置
protocols/   实验协议
scripts/     启动与检查脚本
src/         ROS2 功能包
worlds/      Gazebo 仿真场景
```

## 快速检查 L2 场景

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/check_l2_worlds.sh
```

预期 5 个场景均能加载并运行 12 秒。

## 运行 L1 速度闭环

终端 1：启动 Gazebo 场景。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh S1_single_front_obstacle
```

终端 2：启动 ArduPilot SITL。

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

终端 3：启动 MAVROS。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

终端 4：运行速度状态机。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l1_velocity.sh
```

## 工程路线

总体路线见：

```text
ArduPilot_AKPF_工程实施路线.md
```

当前建议下一步进入 L3：轨迹与实验数据记录层，统一记录位置、速度、速度指令、模式、解锁状态、最近障碍距离等数据。

## 科研方案

AKPF 的科研方案、可行性评估、核心方法和实验设计见：

```text
docs/AKPF_科研方案与可行性评估.md
```
