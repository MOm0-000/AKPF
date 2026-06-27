# L2 ArduPilot 室内场景复现教程

日期：2026-06-11

适用环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Harmonic / Gazebo Sim 8
ArduPilot SITL / ArduCopter
MAVROS2
```

本文目标：在已经完成 L0 基线和 L1 速度闭环的环境上，复现 L2 室内仿真场景与实验协议。最终应完成：

```text
5 个室内 Gazebo 场景可加载
实验协议文件可检查
至少在 S1 场景中跑通 L1 速度闭环任务
结束后无 Gazebo / SITL / MAVROS 残留进程
```

本教程只复现室内场景和实验协议，不加入避障、不加入强化学习、不加入 AKPF。

---

## 快速开始（推荐）

先确认 L1 已编译。随后在 `ld666` 中运行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
bash scripts/open_l2_scene_terminals.sh S1
```

场景可替换为：

```bash
bash scripts/open_l2_scene_terminals.sh S2
bash scripts/open_l2_scene_terminals.sh S3
bash scripts/open_l2_scene_terminals.sh S4
bash scripts/open_l2_scene_terminals.sh S5
```

脚本会打开 Gazebo GUI、ArduPilot SITL、MAVROS 和 L1 速度状态机，用同一套速度闭环验证指定室内场景。终端 2 保持使用：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

任务结束后默认等待 15 秒清理本次相关进程和窗口。只做 SDF 快速检查时仍使用：

```bash
bash scripts/check_l2_worlds.sh
```

常用排错参数：

```bash
bash scripts/open_l2_scene_terminals.sh S5 --no-auto-cleanup
bash scripts/open_l2_scene_terminals.sh --cleanup-only
bash scripts/open_l2_scene_terminals.sh S5 --dry-run --no-cleanup
```

后文保留文件检查和逐终端命令，主要用于定位场景加载、MAVROS 或速度状态机问题。

## 1. 文件说明

L2 文件位于：

```text
/mnt/c/Users/admin/Documents/无人机强化学习 2
```

核心文件：

```text
worlds/l2/S1_single_front_obstacle.sdf
worlds/l2/S2_narrow_gate.sdf
worlds/l2/S3_corridor.sdf
worlds/l2/S4_table_or_low_obstacle.sdf
worlds/l2/S5_corner.sdf
protocols/l2_scenarios.yaml
scripts/start_l2_world.sh
scripts/check_l2_worlds.sh
L2_室内仿真场景与实验协议.md
```

其中：

```text
worlds/l2
```

存放 L2 的 5 个室内 Gazebo world。

```text
protocols/l2_scenarios.yaml
```

定义每个场景的起点、目标点、安全高度、最大时间、障碍物和评价指标。

```text
scripts/start_l2_world.sh
```

用于启动指定 L2 场景，默认启动 `S1_single_front_obstacle`。

```text
scripts/check_l2_worlds.sh
```

用于逐个 headless 加载 5 个 L2 场景，快速检查 SDF、模型路径和 ArduPilot Gazebo 插件路径是否正常。

---

## 2. 注意事项

1. 本流程不使用 conda `COD` 环境。
2. L2 依赖 L1 已经编译完成的 `l1_velocity_control` 包。
3. 每个终端都使用系统 ROS2 Humble 环境。
4. Gazebo 推荐使用软件渲染，脚本中已经默认设置 `LIBGL_ALWAYS_SOFTWARE=1`。
5. 本教程仍然使用 ArduPilot，不使用 PX4。
6. 本教程里的飞行验收只验证速度闭环可控，不要求自动避障。

---

## 3. 手动逐终端验证（排错用）

建议打开 4 个 Ubuntu 终端：

```text
终端 1：Gazebo L2 场景
终端 2：ArduPilot SITL
终端 3：MAVROS
终端 4：L1 速度状态机
```

如果只做场景加载检查，只需要 1 个终端。

---

## 4. 进入项目目录

在任意终端执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
pwd
```

期望输出：

```text
/mnt/c/Users/admin/Documents/无人机强化学习 2
```

如果你的 shell 输出的是：

```text
/mnt/c/users/admin/documents/无人机强化学习 2
```

也可以继续使用，这是同一个 Windows 挂载目录的大小写表现差异。

---

## 5. 检查 L2 文件是否存在

执行：

```bash
find worlds/l2 -maxdepth 1 -type f -name '*.sdf' -print | sort
```

期望输出：

```text
worlds/l2/S1_single_front_obstacle.sdf
worlds/l2/S2_narrow_gate.sdf
worlds/l2/S3_corridor.sdf
worlds/l2/S4_table_or_low_obstacle.sdf
worlds/l2/S5_corner.sdf
```

检查协议文件：

```bash
ls -l protocols/l2_scenarios.yaml
```

期望看到：

```text
protocols/l2_scenarios.yaml
```

检查脚本：

```bash
ls -l scripts/start_l2_world.sh scripts/check_l2_worlds.sh
```

期望两个脚本都有执行权限，例如：

```text
-rwxrwxrwx scripts/start_l2_world.sh
-rwxrwxrwx scripts/check_l2_worlds.sh
```

---

## 6. 检查脚本语法

执行：

```bash
bash -n scripts/start_l2_world.sh
bash -n scripts/check_l2_worlds.sh
```

如果没有任何输出，表示语法检查通过。

---

## 7. 查看实验协议

执行：

```bash
sed -n '1,180p' protocols/l2_scenarios.yaml
```

重点确认以下内容存在：

```text
defaults
success_condition
failure_conditions
metrics
trajectory_record_format
S1_single_front_obstacle
S2_narrow_gate
S3_corridor
S4_table_or_low_obstacle
S5_corner
```

L2 默认安全高度为：

```text
1.2 m 到 2.6 m
```

默认名义飞行高度为：

```text
2.0 m
```

---

## 8. 快速验收：逐个加载 5 个场景

在项目目录执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/check_l2_worlds.sh
```

这个脚本会逐个执行：

```text
gz sim -s -r -v3 worlds/l2/<场景名>.sdf
```

每个场景运行 12 秒。如果 world 能正常加载并持续运行到 timeout，会显示通过。

期望输出：

```text
[L2] Checking S1_single_front_obstacle
[L2] S1_single_front_obstacle: loaded and ran for 12s
[L2] Checking S2_narrow_gate
[L2] S2_narrow_gate: loaded and ran for 12s
[L2] Checking S3_corridor
[L2] S3_corridor: loaded and ran for 12s
[L2] Checking S4_table_or_low_obstacle
[L2] S4_table_or_low_obstacle: loaded and ran for 12s
[L2] Checking S5_corner
[L2] S5_corner: loaded and ran for 12s
```

如果某个场景失败，脚本会打印 `/tmp/l2_<场景名>.log` 的最后 80 行。优先检查：

```text
model://iris_with_gimbal 是否能找到
libArduPilotPlugin.so 是否能加载
SDF XML 是否有拼写错误
```

---

## 9. 单独打开一个 L2 场景

默认打开 S1：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh
```

指定打开 S1：

```bash
./scripts/start_l2_world.sh S1_single_front_obstacle
```

指定打开 S2：

```bash
./scripts/start_l2_world.sh S2_narrow_gate
```

指定打开 S3：

```bash
./scripts/start_l2_world.sh S3_corridor
```

指定打开 S4：

```bash
./scripts/start_l2_world.sh S4_table_or_low_obstacle
```

指定打开 S5：

```bash
./scripts/start_l2_world.sh S5_corner
```

看到类似输出即可认为场景启动成功：

```text
Gazebo Sim Server v8.12.0
Loading SDF world file
World [S1_single_front_obstacle] initialized
```

---

## 10. S1 基线飞行验收

这一节复用 L1 的速度状态机，在 `S1_single_front_obstacle` 中验证：

```text
等待 FCU -> 切 GUIDED -> 解锁 -> 起飞 -> 速度前后左右上下测试 -> 悬停 -> 降落
```

### 10.1 终端 1：启动 S1 Gazebo

打开终端 1，执行：

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

注意：上面这条命令带有 `-s`，表示只启动 Gazebo server，也就是 headless 模式，不会打开可视化窗口。这种方式适合做飞行链路和自动验收。

如果你想看到 Gazebo 可视化界面，使用下面这条命令：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh S1_single_front_obstacle
```

或者直接去掉 `-s`：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"

export GZ_SIM_RESOURCE_PATH="${HOME}/ardupilot_gazebo/models:${HOME}/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
unset __NV_PRIME_RENDER_OFFLOAD || true
unset __GLX_VENDOR_LIBRARY_NAME || true
unset __VK_LAYER_NV_optimus || true

gz sim -r -v3 "worlds/l2/S1_single_front_obstacle.sdf"
```

期望看到：

```text
Gazebo Sim Server v8.12.0
Loading SDF world file[worlds/l2/S1_single_front_obstacle.sdf]
World [S1_single_front_obstacle] initialized
```

也可能看到一些 warning，例如：

```text
XML Element[gz_frame_id], child of element[sensor], not defined in SDF
No scene or camera sensors available
```

这些不是本次 L2 验收的失败条件，只要 world 初始化成功即可继续。

### 10.2 终端 2：启动 ArduPilot SITL

打开终端 2，执行：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

期望看到：

```text
SIM_VEHICLE: Start
SIM_VEHICLE: Run ArduCopter
SIM_VEHICLE: Run MavProxy
Waiting for heartbeat from tcp:127.0.0.1:5760
Detected vehicle 1:1 on link 0
Received 1424 parameters
```

如果停在等待 heartbeat，先确认终端 1 的 Gazebo 场景仍在运行。

### 10.3 终端 3：启动 MAVROS

打开终端 3，执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

期望看到：

```text
FCU URL: udp://:14550@
Plugin distance_sensor ignored
Plugin setpoint_velocity initialized
CON: Got HEARTBEAT, connected. FCU: ArduPilot
```

说明 MAVROS 已经连接 ArduPilot，并且速度控制插件已经加载。

### 10.4 终端 4：运行 L1 速度状态机

打开终端 4，执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l1_velocity.sh
```

期望看到：

```text
L1 velocity node started
Velocity interface: /mavros/setpoint_velocity/cmd_vel (TwistStamped)
Dependencies ready: state, odom, arming, set_mode, takeoff, land
FCU connected
Requested mode: GUIDED
GUIDED already active
Requested arming: true
Vehicle armed
Takeoff command sent to 2.00 m
Takeoff reached 1.89 m, starting velocity test
```

随后会依次执行 12 个速度步骤：

```text
forward +X
hover
backward -X
hover
left +Y
hover
right -Y
hover
up +Z
hover
down -Z
hover
```

完成后期望看到：

```text
Velocity test sequence complete
State -> HOVER
State -> LANDING
Land command sent
Landed or disarmed, mission complete
State -> DONE
L1 velocity mission done, shutting down
```

本次 L2 验收中，S1 已经完整跑通过一轮 L1 速度闭环。

---

## 11. 在其他 L2 场景中复跑

复跑其他场景时，只替换终端 1 的 world 文件。

S2：

```bash
gz sim -s -r -v3 "worlds/l2/S2_narrow_gate.sdf"
```

S3：

```bash
gz sim -s -r -v3 "worlds/l2/S3_corridor.sdf"
```

S4：

```bash
gz sim -s -r -v3 "worlds/l2/S4_table_or_low_obstacle.sdf"
```

S5：

```bash
gz sim -s -r -v3 "worlds/l2/S5_corner.sdf"
```

然后终端 2、3、4 仍然使用同样命令：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l1_velocity.sh
```

注意：L1 速度状态机不是导航任务，不会主动飞到每个场景的目标点。L2 阶段只要求证明场景可以承载稳定起飞和速度控制。真正的目标点导航、避障、奖励函数和评估统计放到后续层。

---

## 12. 结束与清理

推荐按以下顺序停止：

```text
终端 4：L1 速度状态机，通常会自行退出
终端 3：MAVROS，按 Ctrl-C
终端 2：ArduPilot SITL，按 Ctrl-C
终端 1：Gazebo，按 Ctrl-C
```

然后检查残留进程：

```bash
ps -ef | grep -E 'gz sim|gz-sim|sim_vehicle.py|mavros_node|l1_velocity_node|arducopter|ArduCopter|mavproxy.py' | grep -v grep
```

如果没有输出，表示清理完成。

如果有残留进程，先优先回到对应终端按 `Ctrl-C`。如果终端已经关闭，再根据实际 PID 手动结束。

---

## 13. 常见问题

### 13.1 场景打不开，提示找不到模型

典型报错：

```text
Unable to find uri[model://iris_with_gimbal]
```

处理方式：使用 L2 脚本启动。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh S1_single_front_obstacle
```

或手动设置路径：

```bash
export GZ_SIM_RESOURCE_PATH="${HOME}/ardupilot_gazebo/models:${HOME}/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
```

### 13.2 Gazebo OpenGL 报错

典型报错：

```text
GLXBadFBConfig
Failed to create OpenGL context
```

处理方式：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
unset __NV_PRIME_RENDER_OFFLOAD || true
unset __GLX_VENDOR_LIBRARY_NAME || true
unset __VK_LAYER_NV_optimus || true
```

### 13.3 MAVROS 没有连接 FCU

检查 SITL 是否已经看到：

```text
Detected vehicle 1:1 on link 0
```

检查 MAVROS 是否使用了：

```text
fcu_url:=udp://:14550@
```

推荐直接用：

```bash
./scripts/start_mavros_l1.sh
```

### 13.4 L1 状态机一直等待依赖

如果看到：

```text
Waiting deps: state=no odom=no arming=no set_mode=no takeoff=no land=no
```

依次确认：

```bash
ros2 topic list --no-daemon --spin-time 10 | grep /mavros/state
ros2 topic list --no-daemon --spin-time 10 | grep /mavros/local_position/odom
ros2 service list --no-daemon --spin-time 10 | grep /mavros/cmd/arming
ros2 service list --no-daemon --spin-time 10 | grep /mavros/set_mode
```

如果这些接口不存在，说明 MAVROS 未启动成功或未连接 FCU。

### 13.5 重复运行时卡住

先检查残留进程：

```bash
ps -ef | grep -E 'gz sim|gz-sim|sim_vehicle.py|mavros_node|l1_velocity_node|arducopter|ArduCopter|mavproxy.py' | grep -v grep
```

如果上一轮 Gazebo、SITL 或 MAVROS 没有停干净，下一轮可能抢端口或使用旧状态。先清理干净，再按顺序重启：

```text
Gazebo -> SITL -> MAVROS -> L1 状态机
```

---

## 14. L2 成功标准

L2 复现成功应满足：

```text
5 个 SDF 场景文件存在
protocols/l2_scenarios.yaml 存在
scripts/check_l2_worlds.sh 可以逐个加载 5 个场景
S1 场景中 L1 速度闭环可以完成起飞、速度测试、悬停、降落
结束后没有 Gazebo / SITL / MAVROS 残留进程
```

本阶段完成后，可以进入 L3：轨迹与实验数据记录层。
