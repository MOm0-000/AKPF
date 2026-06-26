# L7 ROS2/Gazebo 策略部署复现教程

日期：2026-06-26

本文目标：把 L6 简化环境导出的策略接回 ROS2/MAVROS/Gazebo 链路。L7 不再训练策略，而是做在线推理、任务状态机和与 L5 Safety Shield 的接口对接。

---

## 1. 当前结论

已完成 L7 第一版部署节点：

```text
1. 新增 l7_policy_deployment 包；
2. l7_policy_node 可加载 L6 policy.pt；
3. 使用 MAVROS local odom/pose 捕获 mission_origin；
4. 将当前位置转换为相对起飞点坐标后编码 AKPF 观测；
5. 调用 L6 ActorCritic 做在线推理；
6. 发布 raw velocity 到 /l5/raw_cmd_vel；
7. 保留 L5 Safety Shield 作为最终速度裁剪层；
8. 支持 GUIDED/ARM/TAKEOFF/NAV/HOVER/LAND 状态机。
```

第一版 encoder 使用 L6/L3 的场景抽象几何来保证输入归一化与训练一致。后续再把 encoder 替换为 L4 局部点云/地图特征。

已完成不启动 Gazebo/SITL 的 ROS2 合成冒烟：发布一帧假 `/mavros/local_position/local` 后，节点能捕获 `mission_origin`、进入 `NAVIGATING`，并输出策略速度：

```text
Captured mission_origin=(0.00,0.00,2.00)
State -> NAVIGATING
NAVIGATING nav goal_dist=4.20 cmd=(0.26,-0.23,0.22) inference_ms=0.84
NAVIGATING nav goal_dist=4.20 cmd=(0.26,-0.23,0.22) inference_ms=0.25
```

冒烟结束后已清理 `l7_policy_node`，并复查 Gazebo、ArduPilot、MAVProxy、MAVROS、L3-L5/L7 相关进程无残留。

已完成 L7 接入 L4/L5 多终端链路后的 Gazebo S1-S5 一轮部署验证，均到达目标。每个场景验证结束后均清理 Gazebo、SITL、MAVProxy、MAVROS、bridge、mapper、shield、policy node 相关进程，并确认残留计数为 0。

```text
S1_single_front_obstacle:  GOAL pos=(4.29, 0.45, 2.00), goal_dist=0.46, log=/tmp/l7_S1_20260626_195037
S2_narrow_gate:           GOAL pos=(3.73, 0.14, 2.00), goal_dist=0.49, log=/tmp/l7_S2_20260626_195441
S3_corridor:              GOAL pos=(3.75, 0.05, 2.00), goal_dist=0.46, log=/tmp/l7_S3_20260626_195806
S4_table_or_low_obstacle: GOAL pos=(3.74, -0.08, 2.00), goal_dist=0.47, log=/tmp/l7_S4_20260626_200131
S5_corner:                GOAL pos=(3.35, 2.56, 2.00), goal_dist=0.46, log=/tmp/l7_S5_20260626_200435
```

---

## 2. 编译

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
colcon build --packages-select l6_rl_training l7_policy_deployment --symlink-install
source install/setup.bash
```

期望包含：

```bash
ros2 pkg executables l7_policy_deployment
```

```text
l7_policy_deployment l7_policy_node
```

---

## 3. 接入 L4/L5 多终端链路

下面命令以 S1 为例。S2-S5 只替换 Gazebo world 和 L7 `scenario` 参数：

```text
S1: world=worlds/l4/S1_depth_camera.sdf, scenario=S1_single_front_obstacle
S2: world=worlds/l4/S2_depth_camera.sdf, scenario=S2_narrow_gate
S3: world=worlds/l4/S3_depth_camera.sdf, scenario=S3_corridor
S4: world=worlds/l4/S4_depth_camera.sdf, scenario=S4_table_or_low_obstacle
S5: world=worlds/l4/S5_depth_camera.sdf, scenario=S5_corner
```

### 启动前清理

```bash
pkill -f "arducopter|sim_vehicle.py|mavproxy.py|MAVProxy|gz sim|gzserver|gzclient|ruby.*gz|mavros|l4_gz_pointcloud_bridge_node|l4_pointcloud_mapper_node|l5_safety_shield_node|l7_policy_node" || true
sleep 2
pkill -9 -f "arducopter|sim_vehicle.py|mavproxy.py|MAVProxy|gz sim|gzserver|gzclient|ruby.*gz|mavros|l4_gz_pointcloud_bridge_node|l4_pointcloud_mapper_node|l5_safety_shield_node|l7_policy_node" || true
ps -eo pid,args | grep -E "arducopter|sim_vehicle.py|mavproxy.py|MAVProxy|gz sim|gzserver|gzclient|ruby.*gz|mavros|l4_gz_pointcloud_bridge_node|l4_pointcloud_mapper_node|l5_safety_shield_node|l7_policy_node" | grep -v grep || true
```

### 终端 1：Gazebo GUI

注意：这里不能加 `-s`，需要保留 Gazebo 可视化窗口。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
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

### 终端 6B：L7 policy node

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run l7_policy_deployment l7_policy_node --ros-args \
  -p scenario:=S1_single_front_obstacle \
  -p policy_path:=/tmp/l6_akpf_bc_pass/policy.pt \
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

### 验证结束后清理

```bash
pkill -f "arducopter|sim_vehicle.py|mavproxy.py|MAVProxy|gz sim|gzserver|gzclient|ruby.*gz|mavros|l4_gz_pointcloud_bridge_node|l4_pointcloud_mapper_node|l5_safety_shield_node|l7_policy_node" || true
sleep 2
pkill -9 -f "arducopter|sim_vehicle.py|mavproxy.py|MAVProxy|gz sim|gzserver|gzclient|ruby.*gz|mavros|l4_gz_pointcloud_bridge_node|l4_pointcloud_mapper_node|l5_safety_shield_node|l7_policy_node" || true
ps -eo pid,args | grep -E "arducopter|sim_vehicle.py|mavproxy.py|MAVProxy|gz sim|gzserver|gzclient|ruby.*gz|mavros|l4_gz_pointcloud_bridge_node|l4_pointcloud_mapper_node|l5_safety_shield_node|l7_policy_node" | grep -v grep || true
```

---

## 4. 观察点

```bash
ros2 topic echo /l7/policy_status --once
ros2 topic echo /l5/raw_cmd_vel --once --qos-reliability best_effort
ros2 topic echo /l5/shield_status --once
```

正常日志包含：

```text
Captured mission_origin=(...)
State -> SETTING_GUIDED
State -> ARMING
State -> TAKEOFF
State -> NAVIGATING
NAVIGATING nav goal_dist=... inference_ms=...
```

---

## 5. 当前边界

```text
1. 当前 L7 已完成部署节点、ROS2 合成冒烟和 Gazebo S1-S5 一轮部署验证；
2. 当前 encoder 仍使用 S1-S5 场景抽象几何，不是 L4 点云直接编码；
3. 策略必须经过 L5 Safety Shield；
4. 目标点按 mission_origin 相对坐标解释，不默认 MAVROS local {0,0} 是起飞点；
5. 后续需要补充多次重复统计、Shield 激活次数和推理频率分布。
```
