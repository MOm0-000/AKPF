# L7 ROS2/Gazebo 策略部署复现教程

日期：2026-06-27

本文目标：把 L6 导出的策略接回 ROS2/MAVROS/Gazebo 链路。L7 不训练策略，只负责在线推理、任务状态机、MAVROS 对接，并把原始速度交给 L5 Safety Shield。

默认主线是 L7.1 perception：策略输入来自 L4 深度相机点云/局部地图，不是场景真值几何。只要日志出现 `encoder_mode=geometry`，就说明当前不是 L7.1 点云链路。

---

## 1. 当前结论

L7.1 perception 已完成 Gazebo S1-S5 一轮部署验证。下面结果均使用 L4 深度相机点云/局部地图输入，策略命令经过 L5 Safety Shield，且每个场景结束后均清理 Gazebo、SITL、MAVProxy、MAVROS、bridge、mapper、shield、policy node 相关进程。

```text
S1_single_front_obstacle:  goal_reached pos=(4.15, 0.47, 2.00), goal_dist=0.47, log=artifacts/l7_1_runs_20260627_112228/S1
S2_narrow_gate:           goal_reached pos=(4.00, 0.44, 2.00), goal_dist=0.49, log=artifacts/l7_1_runs_20260627_112644_S2_S5/S2
S3_corridor:              goal_reached pos=(3.75,-0.06, 2.00), goal_dist=0.46, log=artifacts/l7_1_runs_20260627_112644_S2_S5/S3
S4_table_or_low_obstacle: goal_reached pos=(3.76, 0.06, 2.00), goal_dist=0.44, log=artifacts/l7_1_runs_20260627_112644_S2_S5/S4
S5_corner:                goal_reached pos=(3.42, 2.80, 2.00), goal_dist=0.43, log=artifacts/l7_1_runs_20260627_112644_S2_S5/S5
```

推荐使用的 L7.1 perception checkpoint：

```bash
PERCEPTION_POLICY="$PWD/artifacts/tmp_archive_20260627/l6_training/l6_l63_perception_mixed_bc/policy_best.pt"
```

该 checkpoint 的 smoke 信息：

```text
artifacts/tmp_archive_20260627/l6_training/l6_l63_perception_mixed_bc/policy_best.pt: obs_mode=perception, encoder_mode=perception, obs_size=29
```

注意：早期训练输出曾位于 `/tmp/l6_l63_perception_mixed_bc/policy_best.pt`，但 `/tmp` 已清理归档。复现 L7.1 时不要再使用 `/tmp/...` 旧路径。

---

## 2. 最快复现

在 ld666 中运行下面两行即可启动 S5。脚本会打开多个终端窗口；每个终端都会先打印即将执行的命令，再执行对应命令。若 Windows Terminal 能从 WSL 正常启动，脚本会优先使用 Windows Terminal 标签页；否则自动使用 xterm 窗口。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
bash scripts/open_l7_1_perception_terminals.sh S5
```

脚本会依次打开：

```text
T1  Gazebo GUI
T2  ArduPilot SITL
T3  MAVROS
T4  Gazebo PointCloud bridge
T5  L4 local-frame mapper
T6A L5 Safety Shield
T6B L7.1 perception policy node
```

终端 2 的 SITL 命令保持为：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

脚本默认在打开终端 2 后等待 35 秒，再启动 MAVROS 和后续终端。Gazebo 不加 `-s`，会保留可视化窗口。

脚本还会启动一个后台 monitor。检测到 L7 正常降落完成、起飞后异常贴地/坠机、或监控超时后，会等待 15 秒，再自动清理本次仿真相关进程和终端窗口，避免桌面残留 ArduCopter、MAVProxy、Gazebo、L4/L5/L7 窗口。

终端 6B 启动后必须看到：

```text
encoder_mode=perception
policy=.../l6_l63_perception_mixed_bc/policy_best.pt
```

如果看到 `encoder_mode=geometry`，说明跑的是 geometry 对照，不是 L7.1 点云链路。

---

## 3. 脚本参数

基本形式：

```bash
bash scripts/open_l7_1_perception_terminals.sh [S1|S2|S3|S4|S5]
```

参数说明：

| 参数 | 作用 |
| --- | --- |
| `S1|S2|S3|S4|S5` | 位置参数，选择场景；默认 `S1`。 |
| `--scenario SCENE` | 选择场景，等价于位置参数；也支持完整场景名，如 `S5_corner`。 |
| `--sitl-delay SEC` | 终端 2 SITL 打开后等待多久再开 MAVROS；默认 `35`。 |
| `--backend auto|wt|xterm` | 终端打开方式；默认 `auto`，优先可用的 Windows Terminal，不可用时使用 xterm。 |
| `--distro NAME` | Windows Terminal 里使用的 WSL 发行版；默认当前 WSL 或 `ld666`。 |
| `--no-cleanup` | 启动前不自动清理旧 Gazebo/SITL/MAVROS/L4/L5/L7 进程。 |
| `--cleanup-only` | 只清理相关进程和残留终端窗口，不启动仿真。 |
| `--no-auto-cleanup` | 不启动后台 monitor；任务结束后需要手动运行 `--cleanup-only`。 |
| `--auto-cleanup-delay SEC` | 检测到降落完成或坠机事件后等待多久再清理；默认 `45`。 |
| `--post-end-cleanup-delay SEC` | `--auto-cleanup-delay` 的兼容别名。 |
| `--auto-cleanup-timeout SEC` | 如果一直没有检测到终止事件，超过该时间后也清理；默认 `360`，设为 `0` 表示禁用超时清理。 |
| `--dry-run` | 只打印将要执行的命令，不开终端。 |
| `-h` / `--help` | 显示帮助。 |

常用例子：

```bash
# 跑 S1-S5 中任意一个场景
bash scripts/open_l7_1_perception_terminals.sh S1
bash scripts/open_l7_1_perception_terminals.sh S5

# 等 SITL 更久一点再开 MAVROS
bash scripts/open_l7_1_perception_terminals.sh --scenario S5 --sitl-delay 45

# 检测到降落/坠机后等待 60 秒再自动清理
bash scripts/open_l7_1_perception_terminals.sh --scenario S5 --auto-cleanup-delay 60

# 不自动清理，保留终端窗口用于长时间观察
bash scripts/open_l7_1_perception_terminals.sh --scenario S5 --no-auto-cleanup

# 只看脚本会打开哪些命令，不启动终端
bash scripts/open_l7_1_perception_terminals.sh --scenario S5 --dry-run --no-cleanup

# 只清理相关进程和残留终端窗口
bash scripts/open_l7_1_perception_terminals.sh --cleanup-only

# 手动指定 xterm
bash scripts/open_l7_1_perception_terminals.sh --scenario S5 --backend xterm
```

场景对应关系：

```text
S1: world=worlds/l4/S1_depth_camera.sdf, scenario=S1_single_front_obstacle
S2: world=worlds/l4/S2_depth_camera.sdf, scenario=S2_narrow_gate
S3: world=worlds/l4/S3_depth_camera.sdf, scenario=S3_corridor
S4: world=worlds/l4/S4_depth_camera.sdf, scenario=S4_table_or_low_obstacle
S5: world=worlds/l4/S5_depth_camera.sdf, scenario=S5_corner
```

---

## 4. 首次运行或代码更新后编译

如果刚拉取代码、修改过 L4/L5/L6/L7 包，或 `install/` 目录不可用，先编译：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
colcon build --packages-select l4_perception_mapping l5_safety_shield l6_rl_training l7_policy_deployment --symlink-install
source install/setup.bash
```

确认 L7 节点存在：

```bash
ros2 pkg executables l7_policy_deployment
```

期望包含：

```text
l7_policy_deployment l7_policy_node
```

---

## 5. 启动前检查与观察点

脚本默认会先清理旧进程和残留终端窗口。如果只想手动清理，不启动仿真：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
bash scripts/open_l7_1_perception_terminals.sh --cleanup-only
```

多终端启动后，可在任意新终端中检查链路：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Gazebo 应能看到深度相机点云 topic
gz topic -l | grep /l4/depth_camera/points

# MAVROS state 中 connected 应为 true
ros2 topic echo /mavros/state --once

# L4 mapper 应能输出 nearest distance
ros2 topic echo /l4/nearest_distance --once --qos-reliability best_effort

# L7/L5 应能看到 policy 与 shield 状态
ros2 topic echo /l7/policy_status --once
ros2 topic echo /l5/shield_status --once
```

正常 L7 日志包含：

```text
L7 policy node started ... policy=.../l6_l63_perception_mixed_bc/policy_best.pt ... encoder_mode=perception
Captured mission_origin=(...)
State -> SETTING_GUIDED
State -> ARMING
State -> TAKEOFF
State -> NAVIGATING
NAVIGATING nav goal_dist=... inference_ms=... map_age=...
```

如果 L4 地图短时无效，L7.1 会发布 0 速度并输出 `map_not_ready`，不会继续高速冲向障碍。

---

## 6. 手动逐终端启动

本节用于排错或观察单个环节。日常复现优先使用第 2 节的一键脚本。

### 终端 1：Gazebo GUI

注意：这里不能加 `-s`，需要保留 Gazebo 可视化窗口。下面以 S1 为例，跑其他场景时替换 world 文件。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE=1

gz sim -v4 -r worlds/l4/S1_depth_camera.sdf
```

### 终端 2：ArduPilot SITL

必须在真实交互终端中执行并保持打开：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

终端 2 启动后建议等待 35 秒左右，再启动 MAVROS。

### 终端 3：MAVROS

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch mavros node.launch \
  fcu_url:=udp://:14550@ \
  gcs_url:=udp://:14551@ \
  tgt_system:=1 \
  tgt_component:=1 \
  pluginlists_yaml:="$PWD/config/mavros_l1_pluginlists.yaml" \
  config_yaml:=/opt/ros/humble/share/mavros/launch/apm_config.yaml \
  namespace:=mavros
```

### 终端 4：Gazebo 点云 bridge

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run l4_perception_mapping l4_gz_pointcloud_bridge_node --ros-args \
  -p gz_topic:=/l4/depth_camera/points \
  -p ros_topic:=/l4/depth_camera/points \
  -p repack_xyz:=true \
  -p sample_step:=4
```

### 终端 5：local-frame mapper

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run l4_perception_mapping l4_pointcloud_mapper_node --ros-args \
  -p pointcloud_topic:=/l4/depth_camera/points \
  -p odom_topic:=/mavros/local_position/local \
  -p pose_topic:=/mavros/local_position/pose \
  -p use_odom:=true \
  -p transform_to_odom_frame:=true \
  -p output_frame_id:=map \
  -p query_rate_hz:=10.0 \
  -p local_radius_m:=6.0 \
  -p voxel_size_m:=0.15 \
  -p map_memory_s:=90.0 \
  -p max_map_voxels:=80000 \
  -p camera_offset_x:=0.22 \
  -p camera_offset_y:=0.0 \
  -p camera_offset_z:=0.06
```

### 终端 6A：Safety Shield

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run l5_safety_shield l5_safety_shield_node --ros-args \
  -p raw_cmd_topic:=/l5/raw_cmd_vel \
  -p safe_cmd_topic:=/mavros/setpoint_velocity/cmd_vel \
  -p nearest_distance_topic:=/l4/nearest_distance \
  -p nearest_normal_topic:=/l4/nearest_normal \
  -p odom_topic:=/mavros/local_position/local \
  -p state_topic:=/mavros/state \
  -p map_timeout_s:=1.0 \
  -p cmd_timeout_s:=0.5 \
  -p body_radius:=0.35 \
  -p safety_margin:=0.15 \
  -p stop_d_eff:=0.08 \
  -p caution_d_eff:=0.85 \
  -p toward_speed_gain:=0.8 \
  -p max_xy_speed:=0.35 \
  -p max_z_speed:=0.22 \
  -p min_altitude:=1.2 \
  -p max_altitude:=2.6
```

### 终端 6B：L7.1 perception policy node

下面以 S1 为例。跑其他场景时替换 `scenario` 参数。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
PERCEPTION_POLICY="$PWD/artifacts/tmp_archive_20260627/l6_training/l6_l63_perception_mixed_bc/policy_best.pt"

test -f "$PERCEPTION_POLICY" || { echo "missing perception policy: $PERCEPTION_POLICY"; return 1 2>/dev/null || exit 1; }
echo "$PERCEPTION_POLICY"

ros2 run l7_policy_deployment l7_policy_node --ros-args \
  -p scenario:=S1_single_front_obstacle \
  -p policy_path:="$PERCEPTION_POLICY" \
  -p encoder_mode:=perception \
  -p raw_cmd_topic:=/l5/raw_cmd_vel \
  -p odom_topic:=/mavros/local_position/local \
  -p pose_topic:=/mavros/local_position/pose \
  -p state_topic:=/mavros/state \
  -p shield_active_topic:=/l5/shield_active \
  -p nearest_distance_topic:=/l4/nearest_distance \
  -p nearest_normal_topic:=/l4/nearest_normal \
  -p local_cloud_topic:=/l4/local_cloud \
  -p map_timeout_s:=1.0 \
  -p local_cloud_max_points:=2500 \
  -p takeoff_alt:=2.0 \
  -p goal_radius:=0.50 \
  -p max_xy_speed:=0.35 \
  -p max_z_speed:=0.22 \
  -p auto_mission:=true \
  -p require_guided:=true \
  -p mission_timeout_s:=220.0
```

---

## 7. 常见错误判读

```text
UnknownROSArgsError: ['/mnt/c/Users/admin/Documents/无人机强化学习 2']
```

通常是粘贴命令时漏了续行反斜杠 `\`，导致 `cd "/mnt/c/..."` 被拼进了 `ros2 run` 参数。它不是点云失败。

```text
encoder_mode=geometry
```

当前运行的是 geometry checkpoint，例如 `l6_akpf_bc_pass/policy.pt`。它不会使用 L4 点云/局部地图，不是 L7.1 perception 链路。L7.1 主命令使用 `encoder_mode:=perception`，如果误传 geometry checkpoint，会直接报不兼容。

```text
FileNotFoundError: /tmp/l6_l63_perception_mixed_bc/policy_best.pt
```

这是旧路径。当前必须使用 `artifacts/tmp_archive_20260627/l6_training/l6_l63_perception_mixed_bc/policy_best.pt`。

```text
map_not_ready
```

这才表示 L4 点云/局部地图暂时不可用或超时。优先检查终端 4 bridge、终端 5 mapper，以及 `/l4/nearest_distance`、`/l4/local_cloud`。

---

## 8. 当前边界

```text
1. L7.1 perception encoder 已完成 Gazebo S1-S5 一轮实飞验证，S1-S5 均 goal_reached；
2. 策略必须经过 L5 Safety Shield；
3. 目标点按 mission_origin 相对坐标解释，不默认 MAVROS local {0,0} 是起飞点；
4. perception policy 必须由 L6 obs_mode=perception 训练或微调，不能直接拿 geometry policy 使用；
5. 后续需要补充多次重复统计、Shield 激活次数、map_age、point_count、cloud_valid 和推理频率分布。
```

---

## 附录：L7 geometry encoder 对照

下面命令只用于对照旧版 geometry encoder。它加载的是 `obs_mode=akpf` checkpoint，启动日志会出现 `encoder_mode=geometry`。只要看到 `encoder_mode=geometry`，就说明当前不是点云/局部地图避障。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
GEOMETRY_POLICY="$PWD/artifacts/tmp_archive_20260627/l6_training/l6_akpf_bc_pass/policy.pt"

test -f "$GEOMETRY_POLICY" || { echo "missing geometry policy: $GEOMETRY_POLICY"; return 1 2>/dev/null || exit 1; }
echo "$GEOMETRY_POLICY"

ros2 run l7_policy_deployment l7_policy_node --ros-args \
  -p scenario:=S1_single_front_obstacle \
  -p policy_path:="$GEOMETRY_POLICY" \
  -p encoder_mode:=geometry \
  -p raw_cmd_topic:=/l5/raw_cmd_vel \
  -p odom_topic:=/mavros/local_position/local \
  -p pose_topic:=/mavros/local_position/pose \
  -p state_topic:=/mavros/state \
  -p shield_active_topic:=/l5/shield_active \
  -p takeoff_alt:=2.0 \
  -p goal_radius:=0.50 \
  -p max_xy_speed:=0.35 \
  -p max_z_speed:=0.22 \
  -p auto_mission:=true \
  -p require_guided:=true \
  -p mission_timeout_s:=220.0
```
