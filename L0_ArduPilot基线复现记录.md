# L0 ArduPilot 基线复现记录

记录日期：2026-06-10

目标：复现当前已有的 Gazebo + ArduPilot SITL + MAVROS + `offb_node` 状态机闭环，确认现有环境可以完成起飞、矩形航线和降落。

---

## 1. 环境确认

当前环境：

- Linux 用户：`ld666`
- 系统：Ubuntu 22.04.5 LTS
- ROS2：Humble
- Gazebo：Harmonic，`gz sim` 8.12.0
- ArduPilot：Copter SITL，运行时 MAVROS 识别为 `ArduCopter V4.5.7`
- Gazebo 插件：`ardupilot_gazebo`
- ROS2 通信桥：MAVROS
- 状态机工作空间：`~/ws_offboard`
- 状态机节点：`offboard_control offb_node`

启动前检查：

- 未发现残留 `gz sim`、`sim_vehicle.py`、`mavros_node`、`offb_node`、`arducopter` 进程；
- 未发现常用端口 `14550`、`14551`、`9002`、`9003`、`11345`、`11346` 被占用。

---

## 2. 启动命令

### 终端 1：Gazebo

第一次按 NVIDIA 环境变量启动：

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

结果：

- 世界文件开始加载；
- `ArduPilotPlugin` 开始加载；
- GUI 最后失败。

关键错误：

```text
Failed to create OpenGL context
GLXBadFBConfig
```

按文档兜底方案改用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
unset __NV_PRIME_RENDER_OFFLOAD
unset __GLX_VENDOR_LIBRARY_NAME
unset __VK_LAYER_NV_optimus
gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

结果：

- Gazebo GUI 成功启动；
- `iris_runway` world 成功初始化；
- `ArduPilotPlugin` 成功加载；
- IMU、camera、GStreamer camera plugin 等均加载；
- Gazebo 保持运行。

结论：

- 当前机器运行该 Gazebo 场景时，推荐 L0 使用 `LIBGL_ALWAYS_SOFTWARE=1`；
- NVIDIA offload 方式在本次复现中触发 OpenGL/GLX 错误。

---

### 终端 2：ArduPilot SITL

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

观察结果：

- `waf configure` 成功；
- `waf build --target bin/arducopter` 成功；
- `ArduCopter` SITL 启动；
- MavProxy 启动；
- 收到 heartbeat；
- 参数读取成功。

关键输出：

```text
SIM_VEHICLE: Run ArduCopter
SIM_VEHICLE: Run MavProxy
Waiting for heartbeat from tcp:127.0.0.1:5760
Detected vehicle 1:1 on link 0
Received 1424 parameters
```

非阻塞警告：

```text
WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk
```

结论：

- ArduPilot SITL 与 Gazebo JSON 模型连接成功；
- MavProxy 能把 MAVLink 输出到 `udp:127.0.0.1:14550`。

---

### 终端 3：MAVROS

```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@ \
  -p gcs_url:=udp://:14551@
```

观察结果：

- MAVROS 成功启动；
- link `1000` 打开；
- 检测到 remote address `1.1`；
- 收到 ArduPilot heartbeat；
- MAVROS 识别 FCU 为 ArduPilot；
- 飞控版本识别为 `ArduCopter V4.5.7`；
- `setpoint_position`、`setpoint_velocity`、`local_position`、`cmd`、`sys` 等插件加载。

关键输出：

```text
MAVROS UAS via /uas1 started. MY ID 1.191, TARGET ID 1.1
CON: Got HEARTBEAT, connected. FCU: ArduPilot
FCU: ArduCopter V4.5.7
FCU: Frame: QUAD/X
```

已确认的关键 ROS2 topic：

```text
/mavros/state
/mavros/local_position/odom
/mavros/setpoint_position/local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
```

已确认的关键 ROS2 service：

```text
/mavros/set_mode
/mavros/cmd/arming
/mavros/cmd/takeoff
/mavros/cmd/land
```

非阻塞错误：

```text
mavros.distance_sensor: DS: no mapping for sensor id: 0, type: 4, orientation: 25
```

结论：

- MAVROS 和 ArduPilot 连接成功；
- `distance_sensor` 错误与原文档描述一致，本次不影响状态机执行；
- ROS2 CLI 查询 MAVROS 图时需要给 discovery 足够时间，例如 `--spin-time 10`，否则可能短时间内查不到 topic/node。

---

### 终端 4：状态机

```bash
source ~/ws_offboard/install/setup.bash
ros2 run offboard_control offb_node
```

---

## 3. 第一次状态机复现结果

第一次完整成功。

关键过程：

```text
Offboard node started (pure takeoff mode)
FCU connected
Requested mode: GUIDED
GUIDED mode confirmed
Requested arming: true
Arming confirmed
Takeoff command sent to 2.0 m
Takeoff command accepted by FCU
Takeoff reached target altitude (1.96 m)
Waypoint 1 reached
Waypoint 2 reached
Waypoint 3 reached
Waypoint 4 reached
Waypoint 5 reached
Land command sent
Landed, mission complete
Shutting down
```

飞行行为：

1. 连接 FCU；
2. 切换 `GUIDED`；
3. 解锁；
4. 起飞到约 `1.96 m`；
5. 完成 5 个航点；
6. 发送降落；
7. 降落完成；
8. 状态机自动退出。

结论：

- L0 主闭环成功；
- 当前状态机可以驱动 ArduPilot SITL 完成基线任务。

---

## 4. 第二次状态机复现结果

第二次也完整成功，但出现了较长等待。

现象：

- 第二次只重启 `offb_node`，不重启 Gazebo/ArduPilot/MAVROS；
- `offb_node` 启动后较长时间没有打印 `FCU connected`；
- 诊断时发现 `/mavros/state` 的 publisher 和 `offb_node` subscriber 均存在；
- 等待较久后，状态机最终收到状态并继续执行；
- 后续同样完成 GUIDED、arm、takeoff、航点、land。

第二次关键过程：

```text
Offboard node started (pure takeoff mode)
FCU connected
Requested mode: GUIDED
GUIDED mode confirmed
Requested arming: true
Arming confirmed
Takeoff command sent to 2.0 m
Takeoff command accepted by FCU
Takeoff reached target altitude (1.96 m)
Waypoint 1 reached
Waypoint 2 reached
Waypoint 3 reached
Waypoint 4 reached
Waypoint 5 reached
Land command sent
Landed, mission complete
Shutting down
```

结论：

- 第二次任务最终成功；
- 但重复运行时存在 `/mavros/state` 状态更新/ROS2 discovery 等待较慢的问题；
- 后续 L1 或状态机维护阶段应处理这个问题，否则批量实验会被偶发长等待拖慢。

---

## 5. 当前 L0 验收状态

| 项目 | 结果 |
|---|---|
| Gazebo 场景启动 | 通过，需软件渲染 |
| ArduPilot SITL 启动 | 通过 |
| Gazebo-ArduPilot 插件连接 | 通过 |
| MavProxy heartbeat | 通过 |
| MAVROS 连接 FCU | 通过 |
| MAVROS 关键服务可见 | 通过 |
| MAVROS 关键 topic 可见 | 通过，discovery 需等待 |
| 状态机第一次完整任务 | 通过 |
| 状态机第二次完整任务 | 通过，但等待较长 |
| 起飞 | 通过 |
| 矩形航线 | 通过 |
| 降落 | 通过 |

本次完成了 2 次完整任务闭环。严格意义上，如果要求“连续 3 次”，还缺第 3 次；但已经暴露出重复运行时 state/discovery 等待较长的问题，因此建议先记录并在 L1 前修正或规避。

---

## 6. L0 发现的问题

### 问题 1：NVIDIA OpenGL 启动失败

表现：

```text
Failed to create OpenGL context
GLXBadFBConfig
```

当前规避：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

建议：

- L0/L1 阶段优先使用软件渲染；
- 后续如果需要长时间仿真，再单独处理 NVIDIA/WSLg/OpenGL 配置。

### 问题 2：MAVROS distance_sensor 报错刷屏

表现：

```text
mavros.distance_sensor: DS: no mapping for sensor id: 0, type: 4, orientation: 25
```

影响：

- 本次不影响状态机；
- 会污染日志；
- 后续如果要用距离传感器或 rangefinder，需要配置 MAVROS distance sensor 映射。

建议：

- L0 记录为非阻塞项；
- L3/L4 感知阶段再处理；
- 或在 MAVROS 参数中关闭/配置相关 plugin。

### 问题 3：ROS2 discovery 短时间查询不稳定

表现：

- `ros2 topic list` 短等待时可能只看到 `/parameter_events` 和 `/rosout`；
- 增加 `--spin-time 10` 后能看到完整 MAVROS 图。

建议：

检查 ROS2 图时使用：

```bash
ros2 node list --no-daemon --spin-time 10
ros2 topic list --no-daemon --spin-time 10
ros2 service list --no-daemon --spin-time 10
```

### 问题 4：重复运行状态机时 `FCU connected` 等待较长

表现：

- 第一次任务启动后最终正常；
- 第二次只重启状态机时，`FCU connected` 出现明显长等待；
- `/mavros/state` publisher 和 `offb_node` subscriber 能看到，但状态消息到达慢。

可能原因：

- `/mavros/state` 不是高频持续发布；
- 当前状态机订阅 QoS 与 MAVROS state publisher 的行为不完全匹配；
- ROS2 discovery/daemon 等待较慢；
- 任务结束后 MAVROS 状态未主动触发新消息。

建议后续处理：

- 批量复现实验时，每次任务前重启 MAVROS，或增加显式等待；
- L1 设计新桥接节点时，把状态初始化和 QoS 作为重点；
- 考虑使用更合适的 QoS，例如匹配 MAVROS state 的可靠性和 transient local 行为；
- 状态机启动后增加诊断输出：等待 `/mavros/state` publisher、等待服务可用、等待首帧 state。

---

## 7. L0 推荐基线流程

当前最稳流程：

1. 关闭 conda/虚拟环境；
2. 启动 Gazebo，使用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

3. 启动 ArduPilot：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

4. 启动 MAVROS：

```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@ \
  -p gcs_url:=udp://:14551@
```

5. 等待 MAVROS 出现：

```text
CON: Got HEARTBEAT, connected. FCU: ArduPilot
```

6. 运行状态机：

```bash
source ~/ws_offboard/install/setup.bash
ros2 run offboard_control offb_node
```

---

## 8. 对下一层 L1 的影响

L0 已证明当前环境能飞完整基线任务，因此可以进入 L1：ArduPilot 速度控制闭环。

但 L1 前应吸收 L0 的经验：

1. 新状态机必须显式处理 `/mavros/state` 首帧等待；
2. 新状态机应打印每个依赖项是否 ready；
3. 新状态机应避免只靠状态变化触发进入下一阶段；
4. MAVROS `distance_sensor` 报错最好降噪；
5. Gazebo 默认用软件渲染启动，避免 GUI 不稳定；
6. ROS2 CLI 诊断统一使用 `--no-daemon --spin-time 10`。

---

## 9. L0 结论

L0 基线复现通过。

当前环境已经可以完成：

```text
Gazebo iris_runway
  -> ArduPilot SITL
  -> MavProxy
  -> MAVROS
  -> offb_node
  -> GUIDED
  -> arm
  -> takeoff
  -> rectangle waypoints
  -> land
```

该结果说明后续 AKPF 工程可以建立在当前 ArduPilot/MAVROS 基线之上。

L0 主要遗留问题不是飞行链路本身，而是工程稳定性：

- Gazebo GUI 渲染要用软件模式；
- MAVROS distance sensor 报错刷屏；
- ROS2 discovery/状态首帧等待较慢；
- 重复运行状态机的启动等待不够可控。

这些问题应在 L1 速度控制桥接层中优先解决。

