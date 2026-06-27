# L1 ArduPilot 速度闭环复现教程

日期：2026-06-11

适用环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Harmonic / Gazebo Sim 8
ArduPilot SITL / ArduCopter
MAVROS2
```

本文目标：从已经完成 L0 基线的环境出发，复现 L1 速度控制闭环。最终应看到无人机在 Gazebo 中完成：

```text
起飞 -> 前进 -> 后退 -> 左移 -> 右移 -> 上升 -> 下降 -> 悬停 -> 降落
```

本教程只复现速度控制闭环，不加入避障、不加入强化学习、不加入 AKPF。

---

## 快速开始（推荐）

首次运行或代码更新后先编译：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
colcon build --packages-select l1_velocity_control
```

随后一条命令打开 4 个终端：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
bash scripts/open_l1_velocity_terminals.sh
```

脚本会依次启动 Gazebo runway GUI、ArduPilot SITL、MAVROS 和 L1 速度状态机。终端 2 保持使用：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

L1 状态机退出后，脚本默认等待 15 秒并清理本次相关进程和终端窗口。常用参数：

```bash
bash scripts/open_l1_velocity_terminals.sh --sitl-delay 45
bash scripts/open_l1_velocity_terminals.sh --no-auto-cleanup
bash scripts/open_l1_velocity_terminals.sh --cleanup-only
bash scripts/open_l1_velocity_terminals.sh --dry-run --no-cleanup
```

后文的逐终端命令保留为排错入口。

## 1. 文件说明

L1 新增文件位于：

```text
/mnt/c/Users/admin/Documents/无人机强化学习 2
```

核心文件：

```text
src/l1_velocity_control
config/mavros_l1_pluginlists.yaml
scripts/start_mavros_l1.sh
scripts/run_l1_velocity.sh
L1_ArduPilot速度闭环验收记录.md
```

其中：

```text
src/l1_velocity_control
```

是 L1 速度状态机 ROS2 包。

```text
config/mavros_l1_pluginlists.yaml
```

是 L1 专用 MAVROS 插件过滤配置，主要用于禁用 `distance_sensor` 刷屏。

```text
scripts/start_mavros_l1.sh
```

用于按 L1 推荐方式启动 MAVROS。

```text
scripts/run_l1_velocity.sh
```

用于运行 L1 速度闭环状态机，并自动执行 ROS2 graph warmup。

---

## 2. 注意事项

1. 本流程不使用 conda `COD` 环境。
2. 每个终端都使用系统 ROS2 Humble 环境。
3. 如果 `ros2` 命令出现 daemon 超时，诊断命令优先使用 `--no-daemon --spin-time 10`。
4. Gazebo 推荐使用软件渲染启动，否则可能遇到 `GLXBadFBConfig`。
5. MAVROS 推荐用本文的 L1 启动脚本，不推荐裸跑 `mavros_node`。

---

## 3. 手动逐终端验证（排错用）

建议打开 4 个 Ubuntu 终端：

```text
终端 1：Gazebo
终端 2：ArduPilot SITL
终端 3：MAVROS
终端 4：L1 速度状态机
```

如果只是在一个终端里复现，也可以用多个标签页，但不要把四个长运行命令混在同一个 shell 里。

---

## 4. 编译 L1 ROS2 包

在任意终端执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
colcon build --packages-select l1_velocity_control
```

期望看到：

```text
Finished <<< l1_velocity_control
Summary: 1 package finished
```

编译完成后确认节点存在：

```bash
ls -l install/l1_velocity_control/lib/l1_velocity_control/l1_velocity_node
```

期望看到：

```text
install/l1_velocity_control/lib/l1_velocity_control/l1_velocity_node
```

---

## 5. 终端 1：启动 Gazebo

打开终端 1，执行：

```bash
bash -ic 'export LIBGL_ALWAYS_SOFTWARE=1; unset __NV_PRIME_RENDER_OFFLOAD; unset __GLX_VENDOR_LIBRARY_NAME; unset __VK_LAYER_NV_optimus; gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf'
```

这里使用 `bash -ic` 是为了加载用户 shell 中已经配置好的 Gazebo model/plugin 路径。

期望看到类似输出：

```text
Gazebo Sim Server v8.12.0
Loading SDF world file[/home/ld666/ardupilot_gazebo/worlds/iris_runway.sdf]
World [iris_runway] initialized
Loaded system [ArduPilotPlugin]
```

如果看到：

```text
Unable to find uri[model://runway]
Unable to find uri[model://iris_with_gimbal]
```

说明没有加载用户 shell 的 Gazebo 路径。不要用直接 `env LIBGL_ALWAYS_SOFTWARE=1 gz sim ...` 的方式，改用上面的 `bash -ic` 命令。

如果看到：

```text
GLXBadFBConfig
Failed to create OpenGL context
```

说明硬件 OpenGL 路径不稳定，继续使用本文命令中的：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

---

## 6. 终端 2：启动 ArduPilot SITL

打开终端 2，执行：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

期望看到：

```text
SIM_VEHICLE: Start
Run ArduCopter
Run MavProxy
Waiting for heartbeat from tcp:127.0.0.1:5760
Detected vehicle 1:1 on link 0
Received 1424 parameters
```

如果 `sim_vehicle.py` 很快退出，优先确认终端是否支持交互式运行。Codex 或非交互环境中需要 PTY；普通 Ubuntu 终端一般不需要额外处理。

---

## 7. 终端 3：启动 MAVROS

打开终端 3，执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

这个脚本等价于：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros node.launch \
  fcu_url:=udp://:14550@ \
  gcs_url:=udp://:14551@ \
  tgt_system:=1 \
  tgt_component:=1 \
  pluginlists_yaml:="/mnt/c/Users/admin/Documents/无人机强化学习 2/config/mavros_l1_pluginlists.yaml" \
  config_yaml:=/opt/ros/humble/share/mavros/launch/apm_config.yaml \
  namespace:=mavros
```

期望看到：

```text
Plugin distance_sensor ignored
Plugin setpoint_velocity created
Plugin setpoint_velocity initialized
MAVROS UAS via /uas1 started
CON: Got HEARTBEAT, connected. FCU: ArduPilot
FCU: ArduCopter V4.5.7
FCU: Frame: QUAD/X
```

重点确认：

```text
Plugin distance_sensor ignored
```

如果没有这行，并且看到大量：

```text
mavros.distance_sensor: DS: no mapping for sensor id: 0, type: 4, orientation: 25
```

说明没有按 L1 配置启动 MAVROS。停止 MAVROS 后重新执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

---

## 8. 可选检查：确认 MAVROS 速度接口

新开一个临时终端，执行：

```bash
source /opt/ros/humble/setup.bash
ros2 topic info -v /mavros/setpoint_velocity/cmd_vel --no-daemon --spin-time 10
```

期望看到：

```text
Type: geometry_msgs/msg/TwistStamped
Subscription count: 1
Node name: setpoint_velocity
Node namespace: /mavros
Reliability: BEST_EFFORT
Durability: VOLATILE
```

再检查无时间戳速度接口：

```bash
source /opt/ros/humble/setup.bash
ros2 topic info -v /mavros/setpoint_velocity/cmd_vel_unstamped --no-daemon --spin-time 10
```

期望看到：

```text
Type: geometry_msgs/msg/Twist
Subscription count: 1
Node name: setpoint_velocity
Node namespace: /mavros
Reliability: BEST_EFFORT
Durability: VOLATILE
```

L1 默认使用：

```text
/mavros/setpoint_velocity/cmd_vel
geometry_msgs/msg/TwistStamped
```

原因是 `TwistStamped` 带时间戳，更适合后续科研记录、frame 语义和安全裁剪。

---

## 9. 可选检查：确认 state 和 odom

查看 `/mavros/state`：

```bash
source /opt/ros/humble/setup.bash
ros2 topic info -v /mavros/state --no-daemon --spin-time 10
```

期望看到：

```text
Type: mavros_msgs/msg/State
Publisher count: 1
Node name: sys
Node namespace: /mavros
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL
```

查看 odom：

```bash
source /opt/ros/humble/setup.bash
ros2 topic info -v /mavros/local_position/odom --no-daemon --spin-time 10
```

期望看到：

```text
Type: nav_msgs/msg/Odometry
Publisher count: 1
```

如果这些检查偶尔只看到 `/rosout` 和 `/parameter_events`，不要立刻判定 MAVROS 挂了。当前 WSL/ROS2/FastDDS 环境里 discovery 偶尔较慢，继续执行下一节的 L1 运行脚本即可，因为脚本内已经加入 graph warmup。

---

## 10. 终端 4：运行 L1 速度状态机

打开终端 4，执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l1_velocity.sh
```

这个脚本会做三件事：

1. 加载 ROS2 Humble；
2. 加载当前工程的 `install/setup.bash`；
3. 并行执行两条 no-daemon graph warmup 命令，然后启动 L1 节点。

脚本内部核心命令为：

```bash
source /opt/ros/humble/setup.bash
source "/mnt/c/Users/admin/Documents/无人机强化学习 2/install/setup.bash"

(ros2 node list --no-daemon --spin-time 10 >/tmp/l1_velocity_node_warmup.log 2>&1 || true) &
(ros2 topic list -t --no-daemon --spin-time 10 >/tmp/l1_velocity_topic_warmup.log 2>&1 || true) &

ros2 run l1_velocity_control l1_velocity_node --ros-args \
  -p use_stamped_cmd_vel:=true \
  -p mission_timeout_s:=180.0
```

---

## 11. L1 正常输出

启动后首先应看到：

```text
L1 velocity node started
Velocity interface: /mavros/setpoint_velocity/cmd_vel (TwistStamped)
Limits: v_xy=0.35 m/s, v_z=0.20 m/s, publish_rate=20.0 Hz
```

随后等待依赖：

```text
Waiting deps: state=no odom=no arming=no set_mode=no takeoff=no land=no
```

正常情况下，graph warmup 后会变成：

```text
Dependencies ready: state, odom, arming, set_mode, takeoff, land
State -> WAITING_FCU
FCU connected
State -> SETTING_GUIDED
Requested mode: GUIDED
Set mode response: mode_sent=yes
GUIDED already active
State -> ARMING
Requested arming: true
Arm response: success=yes
Vehicle armed
State -> TAKEOFF
Takeoff command sent to 2.00 m
Takeoff response: success=yes
```

起飞成功后：

```text
Takeoff reached 1.89 m, starting velocity test
State -> VELOCITY_TEST
```

速度测试阶段会依次看到：

```text
Velocity step 1/12: forward +X
Velocity step complete: forward +X

Velocity step 3/12: backward -X
Velocity step complete: backward -X

Velocity step 5/12: left +Y
Velocity step complete: left +Y

Velocity step 7/12: right -Y
Velocity step complete: right -Y

Velocity step 9/12: up +Z
Velocity step complete: up +Z

Velocity step 11/12: down -Z
Velocity step complete: down -Z
```

最后应看到：

```text
Velocity test sequence complete
State -> HOVER
State -> LANDING
Land command sent
Landed or disarmed, mission complete
State -> DONE
L1 velocity mission done, shutting down
```

看到以上内容即表示 L1 复现成功。

---

## 12. L1 验收标准

必须满足：

```text
1. MAVROS 成功连接 ArduPilot；
2. setpoint_velocity 插件加载成功；
3. L1 节点能进入 GUIDED；
4. L1 节点能解锁；
5. L1 节点能起飞到约 2 m；
6. L1 节点能用速度指令完成 X/Y/Z 三轴动作；
7. L1 节点能悬停；
8. L1 节点能降落；
9. 节点最终自动退出；
10. 没有出现失控或持续爬升/漂移。
```

实际验收中，成功日志里位置变化大致如下：

```text
forward +X:  x 从约 -0.08 到约 0.34
backward -X: x 从约 0.62 到约 0.19
left +Y:     y 从约 0.01 到约 0.45
right -Y:    y 从约 0.68 到约 0.23
up +Z:       z 从约 2.01 到约 2.19
down -Z:     z 从约 2.32 到约 2.13
```

位置变化不需要完全一致，但方向应一致，且高度最终应能回到降落流程。

---

## 13. 常见问题

### 13.1 Gazebo 找不到模型

报错：

```text
Unable to find uri[model://runway]
Unable to find uri[model://iris_with_gimbal]
```

处理：

```bash
bash -ic 'export LIBGL_ALWAYS_SOFTWARE=1; unset __NV_PRIME_RENDER_OFFLOAD; unset __GLX_VENDOR_LIBRARY_NAME; unset __VK_LAYER_NV_optimus; gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf'
```

不要直接执行：

```bash
env LIBGL_ALWAYS_SOFTWARE=1 gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

因为这种方式可能没有加载用户 shell 里的 Gazebo 路径。

### 13.2 Gazebo OpenGL 报错

报错：

```text
GLXBadFBConfig
Failed to create OpenGL context
```

处理：使用软件渲染启动 Gazebo：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

本文的 Gazebo 启动命令已经包含该设置。

### 13.3 MAVROS distance_sensor 刷屏

报错：

```text
mavros.distance_sensor: DS: no mapping for sensor id: 0, type: 4, orientation: 25
```

处理：使用 L1 MAVROS 启动脚本：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

确认日志中出现：

```text
Plugin distance_sensor ignored
```

### 13.4 L1 一直等待依赖

现象：

```text
Waiting deps: state=no odom=no arming=no set_mode=no takeoff=no land=no
```

处理 1：优先使用脚本运行，而不是手动裸跑节点：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l1_velocity.sh
```

处理 2：另开一个终端执行 graph warmup：

```bash
source /opt/ros/humble/setup.bash
ros2 node list --no-daemon --spin-time 10
ros2 topic list -t --no-daemon --spin-time 10
```

处理 3：如果仍然不行，按顺序重启：

```text
1. 停止 L1 节点
2. 停止 MAVROS
3. 重新执行 ./scripts/start_mavros_l1.sh
4. 等 MAVROS 出现 CON: Got HEARTBEAT
5. 重新执行 ./scripts/run_l1_velocity.sh
```

### 13.5 `ros2 daemon` 超时

报错：

```text
TimeoutError: [Errno 110] Connection timed out
```

处理：诊断命令不要依赖 daemon，改用：

```bash
ros2 node list --no-daemon --spin-time 10
ros2 topic list -t --no-daemon --spin-time 10
ros2 topic info -v /mavros/state --no-daemon --spin-time 10
```

### 13.6 起飞后很快进入 FAILSAFE

如果你人为把：

```text
mission_timeout_s
```

设置得太短，例如 10 秒，L1 会在起飞阶段进入 failsafe landing。这是正常保护逻辑。

正常复现使用：

```bash
./scripts/run_l1_velocity.sh
```

默认超时为：

```text
180.0 s
```

如需手动加长：

```bash
MISSION_TIMEOUT_S=240.0 ./scripts/run_l1_velocity.sh
```

---

### 13.7 `/opt/ros/humble/setup.bash: AMENT_TRACE_SETUP_FILES: unbound variable`

现象：

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

原因：脚本如果在 `source /opt/ros/humble/setup.bash` 之前启用了 `set -u`，ROS2 的 setup 脚本会把未定义变量当成致命错误。当前脚本已经修正为先 source ROS，再启用 `set -u`。

处理：重新拉取或确认脚本开头类似下面这样：

```bash
#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u
```

另外，启动命令必须是一整条：

```bash
./scripts/start_mavros_l1.sh
```

不要写成：

```bash
.
/scripts/start_mavros_l1.sh
```

也不要写成：

```bash
. /scripts/start_mavros_l1.sh
```

---

## 14. 结束与清理

L1 正常完成后，终端 4 会自动退出。

然后按顺序停止：

```text
终端 3：Ctrl+C 停止 MAVROS
终端 2：Ctrl+C 停止 ArduPilot SITL
终端 1：Ctrl+C 停止 Gazebo
```

最后检查是否还有残留进程：

```bash
ps -ef | grep -E 'gz sim|gz-sim|sim_vehicle.py|mavros_node|l1_velocity_node|arducopter|ArduCopter' | grep -v grep
```

如果没有输出，说明已经清理干净。

如果 Gazebo 仍然残留，可以先找父进程：

```bash
ps -ef | grep -E 'gz sim|gz-sim' | grep -v grep
```

然后对 `gz sim -v4 -r ...` 那个父进程发送 SIGINT：

```bash
kill -INT <PID>
```

不要优先使用 `kill -9`，除非正常中断无效。

---

## 15. 一键命令速查

### 终端 1

```bash
bash -ic 'export LIBGL_ALWAYS_SOFTWARE=1; unset __NV_PRIME_RENDER_OFFLOAD; unset __GLX_VENDOR_LIBRARY_NAME; unset __VK_LAYER_NV_optimus; gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf'
```

### 终端 2

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

### 终端 3

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

### 终端 4

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l1_velocity.sh
```

---

## 16. 本层结论

L1 复现成功后，说明当前工程已经具备：

```text
ROS2 velocity command
  -> MAVROS setpoint_velocity
  -> ArduPilot GUIDED velocity control
  -> Gazebo simulated UAV motion
```

这意味着后续 AKPF、RL 和 Safety Shield 不需要再直接处理位置 setpoint，可以统一输出速度指令：

```text
[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]
```

L1 是后续避障算法进入真实仿真闭环前的速度控制桥接层。
