# L3 ArduPilot 真值几何 AKPF 复现教程

日期：2026-06-12

适用环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Harmonic / Gazebo Sim 8
ArduPilot SITL / ArduCopter
MAVROS2
```

本文目标：在 L2 室内场景基础上，复现 L3 真值几何 AKPF 非学习版局部导航器。L3 不使用点云、不使用 ESDF、不使用强化学习，而是直接使用 L2 场景中已知的障碍物几何，生成 MAVROS 速度指令。

---

## 快速开始（推荐）

首次运行或代码更新后先编译：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
colcon build --packages-select l3_akpf_navigation
```

随后用一条命令打开 L3 的 4 个终端：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
bash scripts/open_l3_akpf_terminals.sh S1
```

可选场景：

```bash
bash scripts/open_l3_akpf_terminals.sh S2
bash scripts/open_l3_akpf_terminals.sh S3
bash scripts/open_l3_akpf_terminals.sh S4
bash scripts/open_l3_akpf_terminals.sh S5
bash scripts/open_l3_akpf_terminals.sh S6
bash scripts/open_l3_akpf_terminals.sh S8
bash scripts/open_l3_akpf_terminals.sh S9
```

脚本会依次打开 Gazebo GUI、ArduPilot SITL、MAVROS 和 L3 AKPF 节点。终端 2 保持使用：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

L3 节点退出后，脚本默认等待 15 秒清理本次相关进程和终端窗口。常用排错参数：

```bash
bash scripts/open_l3_akpf_terminals.sh S5 --no-auto-cleanup
bash scripts/open_l3_akpf_terminals.sh --cleanup-only
bash scripts/open_l3_akpf_terminals.sh S5 --dry-run --no-cleanup
```

后文保留逐终端命令，用于检查 MAVROS、AKPF 参数和场景日志。

## 1. L3 做什么

L3 的输入：

```text
L2 场景名称
L2 场景内已知 box 障碍物几何
/mavros/local_position/odom
/mavros/state
目标点
```

L3 的输出：

```text
/mavros/setpoint_velocity/cmd_vel
```

内部流程：

```text
已知障碍物几何
  -> AABB 距离与法向查询
  -> 目标吸引
  -> 几何排斥
  -> 动力学有效距离
  -> 旋度偏置
  -> 气动代理风险
  -> 速度裁剪与加速度限制
  -> MAVROS 速度指令
```

---

## 2. 新增文件

项目根目录：

```text
/mnt/c/Users/admin/Documents/无人机强化学习 2
```

L3 ROS2 包：

```text
src/l3_akpf_navigation
```

核心节点：

```text
src/l3_akpf_navigation/src/l3_akpf_node.cpp
```

运行脚本：

```text
scripts/run_l3_akpf.sh
```

验收记录：

```text
L3_真值几何AKPF验收记录.md
```

---

## 3. L3 支持的场景

L3 当前内置的几何与 `protocols/l2_scenarios.yaml` 对齐：

```text
S1_single_front_obstacle
S2_narrow_gate
S3_corridor
S4_table_or_low_obstacle
S5_corner
S6_cluttered_boxes
S8_vertical_constraint
S9_multi_corner
```

当前已验收：

```text
S1_single_front_obstacle
S2_narrow_gate
S3_corridor
S4_table_or_low_obstacle
S5_corner
S6_cluttered_boxes
S8_vertical_constraint
S9_multi_corner
```

S1-S5 属于 L3 基础场景，S6/S8/S9 属于 L3.5 进阶真值几何压力测试。S1/S5/S9 等问题场景当前通过的是通用候选速度、通用几何局部目标、局部目标保持和终端区处理，不依赖场景专用局部目标点。

---

## 4. 编译 L3

进入项目目录：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
```

加载 ROS2 环境：

```bash
source /opt/ros/humble/setup.bash
```

编译 L3 包：

```bash
colcon build --packages-select l3_akpf_navigation
```

期望看到：

```text
Finished <<< l3_akpf_navigation
Summary: 1 package finished
```

确认节点存在：

```bash
source install/setup.bash
ros2 pkg executables l3_akpf_navigation
```

期望输出：

```text
l3_akpf_navigation l3_akpf_node
```

---

## 5. L3 节点空跑检查

不启动 MAVROS 时，可以先确认节点能启动并等待依赖：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 3s ros2 run l3_akpf_navigation l3_akpf_node --ros-args -p scenario:=S1_single_front_obstacle
```

期望看到：

```text
L3 AKPF node started
Scenario: S1_single_front_obstacle
AKPF terms: repulsion=yes kino=yes curl=yes aero=yes
Waiting deps: state=no odom=no arming=no set_mode=no takeoff=no land=no
```

`timeout` 退出码为 `124` 是正常的，因为这里没有启动 MAVROS。

---

## 6. 手动逐终端实飞复现（排错用）

建议打开 4 个终端：

```text
终端 1：Gazebo L2 场景
终端 2：ArduPilot SITL
终端 3：MAVROS
终端 4：L3 AKPF 节点
```

### 6.1 终端 1：启动 S1 Gazebo

headless 模式：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"

export GZ_SIM_RESOURCE_PATH="${HOME}/ardupilot_gazebo/models:${HOME}/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
unset __NV_PRIME_RENDER_OFFLOAD || true
unset __GLX_VENDOR_LIBRARY_NAME || true
unset __VK_LAYER_NV_optimus || true

gz sim -s -r -v3 "worlds/l2/S1_single_front_obstacle.sdf"
```

如果要打开可视化界面：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh S1_single_front_obstacle
```

### 6.2 终端 2：启动 ArduPilot SITL

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

看到以下内容即可继续：

```text
Detected vehicle 1:1 on link 0
Received 1424 parameters
```

### 6.3 终端 3：启动 MAVROS

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

看到以下内容即可继续：

```text
Plugin setpoint_velocity initialized
CON: Got HEARTBEAT, connected. FCU: ArduPilot
```

### 6.4 终端 4：运行 L3 AKPF

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
SCENARIO=S1_single_front_obstacle MISSION_TIMEOUT_S=120 ./scripts/run_l3_akpf.sh
```

期望看到：

```text
L3 AKPF node started
Scenario: S1_single_front_obstacle, goal=(4.20, 0.00, 2.00), obstacles=5
Dependencies ready: state, odom, arming, set_mode, takeoff, land
FCU connected
GUIDED already active
Vehicle armed
Takeoff command sent to 2.00 m
Takeoff reached ... starting AKPF navigation
```

S1 成功时会出现类似轨迹日志：

```text
AKPF pos=(0.01, 0.01, 1.86) ... cmd=(0.35, 0.28, 0.06)
AKPF pos=(3.46, 1.66, 2.00) ... cmd=(0.44, -0.08, -0.00)
AKPF pos=(4.18, 1.21, 2.00) ... cmd=(0.01, -0.45, 0.00)
Goal reached: pos=(4.20, 0.47, 2.00), goal_dist=0.47, min_clearance=0.74
State -> HOVER
State -> LANDING
Landed or disarmed, mission complete
State -> DONE
L3 AKPF mission done, shutting down
```

---

## 7. 运行其他场景

终端 1 替换 world：

```bash
gz sim -s -r -v3 "worlds/l2/S2_narrow_gate.sdf"
```

终端 4 替换场景参数：

```bash
SCENARIO=S2_narrow_gate MISSION_TIMEOUT_S=140 ./scripts/run_l3_akpf.sh
```

其他场景同理：

```bash
SCENARIO=S3_corridor MISSION_TIMEOUT_S=140 ./scripts/run_l3_akpf.sh
SCENARIO=S4_table_or_low_obstacle MISSION_TIMEOUT_S=140 ./scripts/run_l3_akpf.sh
SCENARIO=S5_corner MISSION_TIMEOUT_S=160 ./scripts/run_l3_akpf.sh
```

注意：S1-S5 已作为 L3 基础场景完成验收。S6/S8/S9 的详细复现命令和本次验收日志见 `L3_5_进阶真值几何AKPF压力测试复现教程.md`。每次只换一个场景，记录是否出现局部极小、贴障碍、过度保守或超时。

---

## 8. 参数开关

脚本支持用环境变量开关 AKPF 项：

```bash
ENABLE_REPULSION=true
ENABLE_KINO=true
ENABLE_CURL=true
ENABLE_AERO=true
```

只验证目标吸引：

```bash
ENABLE_REPULSION=false ENABLE_KINO=false ENABLE_CURL=false ENABLE_AERO=false ./scripts/run_l3_akpf.sh
```

验证几何排斥：

```bash
ENABLE_REPULSION=true ENABLE_KINO=false ENABLE_CURL=false ENABLE_AERO=false ./scripts/run_l3_akpf.sh
```

验证动力学有效距离：

```bash
ENABLE_REPULSION=true ENABLE_KINO=true ENABLE_CURL=false ENABLE_AERO=false ./scripts/run_l3_akpf.sh
```

验证旋度偏置：

```bash
ENABLE_REPULSION=true ENABLE_KINO=true ENABLE_CURL=true ENABLE_AERO=false ./scripts/run_l3_akpf.sh
```

完整 L3：

```bash
ENABLE_REPULSION=true ENABLE_KINO=true ENABLE_CURL=true ENABLE_AERO=true ./scripts/run_l3_akpf.sh
```

### 8.1 通用终端区参数

S5 曾经在目标附近出现引力与多面墙体斥力抵消的问题。当前处理方式不是给 S5 增加固定局部目标点，而是在所有场景中使用同一套终端区逻辑：

```text
terminal_radius = 1.40
terminal_goal_boost = 0.24
terminal_repulsion_relief = 0.65
```

含义：

```text
terminal_radius：进入目标终端区的半径；
terminal_goal_boost：终端区内额外增强目标方向速度；
terminal_repulsion_relief：终端区内适度软化障碍斥力，避免目标附近势场互相抵消。
```

同时，房间边界墙已经在 S1-S5 中统一作为障碍物参与距离查询。S5 的通过不依赖任何场景专用绕行点。

### 8.2 通用几何局部目标参数

S1 曾经在绕过单障碍后形成周期震荡。当前处理方式不是给 S1 增加固定路点，而是在所有场景中使用同一套几何局部目标逻辑：

```text
enable_local_target = true
local_target_clearance = 0.08
local_target_inflate_extra = 0.25
local_target_sample_step = 0.20
candidate_comfort_margin = 0.38
recovery_progress_floor = 0.05
```

含义：

```text
enable_local_target：启用自动几何局部目标；
local_target_clearance：线段可见性需要保留的额外净空；
local_target_inflate_extra：在机体半径和安全边界外继续膨胀障碍角点；
local_target_sample_step：线段净空采样步长；
candidate_comfort_margin：候选方向超过该安全裕度后不再继续奖励远离障碍；
recovery_progress_floor：贴近障碍恢复时仍要求候选方向对目标有正进展。
```

这个局部目标由障碍物几何自动生成，不读取场景名，不包含 S1 或 S5 专用坐标。

---

## 9. 清理

推荐按顺序停止：

```text
终端 4：L3 AKPF，通常会自行退出
终端 3：MAVROS，按 Ctrl-C
终端 2：ArduPilot SITL，按 Ctrl-C
终端 1：Gazebo，按 Ctrl-C
```

检查残留进程：

```bash
ps -ef | grep -E 'gz sim|gz-sim|sim_vehicle.py|mavros_node|l1_velocity_node|l3_akpf_node|arducopter|ArduCopter|mavproxy.py' | grep -v grep
```

如果没有输出，表示清理完成。

---

## 10. 当前成功标准

L3 当前最小成功标准：

```text
l3_akpf_navigation 可以编译
l3_akpf_node 可以启动并等待 MAVROS 依赖
S1 中可以起飞
S1 中可以绕开正前障碍
S1 中可以到达目标半径 0.5 m 内
S1 中可以悬停、降落、DONE
S5 中不使用场景专用局部目标点也能到达目标
S5 中可以悬停、降落、DONE
结束后无残留进程
```

当前 S1-S5 已作为 L3 基础场景完成验收，S6/S8/S9 已作为 L3.5 进阶真值几何压力测试完成验收。下一步建议进入 L4 点云/局部地图 AKPF，而不是继续在真值几何阶段扩写场景专用规则。
