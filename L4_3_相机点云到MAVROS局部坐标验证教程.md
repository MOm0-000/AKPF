# L4.3 相机点云到 MAVROS 局部坐标验证教程

日期：2026-06-23

本文目标：在 L4.2 Gazebo 深度相机点云已经能进入 ROS2 后，把相机坐标系点云转换到 MAVROS local ENU 坐标系，并让 L3 AKPF 可以选择 `perception_map` 作为最近障碍距离来源。

本文现在推荐先用一键多终端脚本快速复现，再用后文逐终端命令排错。Gazebo 必须保持可视化，方便用户亲眼观察仿真结果；手动或脚本启动都不要给 Gazebo 加 `-s`。

---

## 快速开始（推荐）

首次运行或代码更新后先编译：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-select l1_velocity_control l3_akpf_navigation l4_perception_mapping
```

随后用一条命令打开 L4.3 的 6 个终端：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
bash scripts/open_l4_3_perception_terminals.sh S1
```

可选场景：

```bash
bash scripts/open_l4_3_perception_terminals.sh S2
bash scripts/open_l4_3_perception_terminals.sh S3
bash scripts/open_l4_3_perception_terminals.sh S4
bash scripts/open_l4_3_perception_terminals.sh S5
```

脚本会依次打开 Gazebo GUI、ArduPilot SITL、MAVROS、Gazebo 点云 bridge、local-frame mapper 和感知版 AKPF。终端 2 保持使用：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

AKPF 节点退出后，脚本默认等待 15 秒清理本次相关进程和终端窗口。常用排错参数：

```bash
bash scripts/open_l4_3_perception_terminals.sh S5 --no-auto-cleanup
bash scripts/open_l4_3_perception_terminals.sh --cleanup-only
bash scripts/open_l4_3_perception_terminals.sh S5 --dry-run --no-cleanup
```

后文保留逐终端命令，用于单独检查点云、bridge、mapper 和 AKPF。

## 1. 当前结论

已完成：

```text
1. l4_pointcloud_mapper_node 支持 transform_to_odom_frame；
2. mapper 可使用 MAVROS local_position/pose，Odometry 作为兜底；
3. mapper 可把相机点云按固定相机外参变换到 local ENU；
4. mapper 发布 /l4/local_cloud、/l4/nearest_distance、/l4/nearest_point、/l4/nearest_normal；
5. l3_akpf_node 支持 distance_source:=truth_geometry / perception_map；
6. L3 perception_map 模式可订阅 /l4/local_cloud 并用点云最近点替代真值 box 最近距离。
```

2026-06-23 实跑验证结果：

```text
Gazebo GUI: 正常启动 L4_S1_depth_camera
Gazebo topic: /l4/depth_camera/points 存在
SITL/MAVProxy: 使用 --map --console 后保持交互，进入 STABILIZE
MAVROS: CON: Got HEARTBEAT, connected. FCU: ArduPilot
Bridge: Bridged PointCloud2: width=1200 height=1 point_step=12
Mapper: L4 query pos=(-0.01, -0.01, -0.03) input=1200 local=441 nearest=0.91
```

注意：`sim_vehicle.py --map --console` 必须在真实交互终端中运行，不要后台化、管道化或让 stdin 断开，否则 MAVProxy 可能会自动退出。

2026-06-23 坠机原因与通用修复：

```text
原因：perception_map 只用当前帧点云时，绕到障碍物侧边后局部点云会短暂看不见近处表面，
      最近距离可能从真实贴障状态跳到 2m 以上，AKPF 继续向目标回切，撞到 front_block 边角。
修复：
1. L4 mapper 默认保留 90 秒局部体素记忆，/l4/local_cloud 不再只来自当前帧；
2. L3 增加 emergency_d_eff 近障恢复模式，d_eff 过小时先脱离障碍；
3. L3 恢复速度改为“法向脱离 + 目标方向切向绕行”，避免只后退而不绕行；
4. L3 恢复模式加入 recovery_exit_d_eff、recovery_exit_path_margin 和 recovery_min_duration_s 滞回；
5. L3 候选速度筛选使用通用安全裕度，不绑定 S1 障碍位置，S1-S5 使用同一套逻辑；
6. safety violation 同时检查 d_eff，机体半径重叠风险会更早触发保护。
```

2026-06-25 感知版 AKPF 实跑结果：

```text
S1_single_front_obstacle: GOAL pos=(4.12, 0.47, 2.00), goal_dist=0.48, min_clearance=0.75
S2_narrow_gate:          GOAL pos=(3.76, -0.23, 2.02), goal_dist=0.49, min_clearance=0.79
S3_corridor:             GOAL pos=(3.72, 0.00, 2.01), goal_dist=0.48, min_clearance=1.00
S4_table_or_low_obstacle:GOAL pos=(3.74, -0.00, 2.04), goal_dist=0.46, min_clearance=1.21
S5_corner:               GOAL pos=(3.34, 2.63, 2.00), goal_dist=0.46, min_clearance=0.78
```

---

## 2. 编译

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-select l1_velocity_control l3_akpf_navigation l4_perception_mapping
source install/setup.bash
```

期望节点：

```bash
ros2 pkg executables l4_perception_mapping
ros2 pkg executables l3_akpf_navigation
```

期望包含：

```text
l4_perception_mapping l4_gz_pointcloud_bridge_node
l4_perception_mapping l4_pointcloud_mapper_node
l4_perception_mapping l4_synthetic_cloud_node
l3_akpf_navigation l3_akpf_node
```

---

## 3. L4.3 手动逐终端验证顺序（排错用）

### 终端 1：启动可视化 Gazebo

不要加 `-s`。本终端必须保持 Gazebo GUI 可见。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -v4 -r worlds/l4/S1_depth_camera.sdf
```

S2-S5 可把 world 文件替换为：

```text
worlds/l4/S2_depth_camera.sdf
worlds/l4/S3_depth_camera.sdf
worlds/l4/S4_depth_camera.sdf
worlds/l4/S5_depth_camera.sdf
```

另开临时命令检查 Gazebo 点云：

```bash
gz topic -l | grep /l4/depth_camera/points
```

期望包含：

```text
/l4/depth_camera/points
```

### 终端 2：启动 ArduPilot SITL

必须在真实交互终端中执行并保持打开：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

期望看到：

```text
Loaded module console
Loaded module map
Detected vehicle 1:1
STABILIZE>
```

### 终端 3：启动 MAVROS

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

期望看到：

```text
CON: Got HEARTBEAT, connected. FCU: ArduPilot
```

临时检查：

```bash
source /opt/ros/humble/setup.bash
source "/mnt/c/Users/admin/Documents/无人机强化学习 2/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
```

### 终端 4：启动 Gazebo 点云 bridge

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

期望看到：

```text
Bridged PointCloud2: width=1200 height=1 point_step=12
```

### 终端 5：启动 local-frame mapper

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

期望看到：

```text
L4 pointcloud mapper started: ... use_odom=true transform=odom_frame
L4 query pos=(..., ..., ...) input=1200 local=... nearest=...
```

临时检查：

```bash
source /opt/ros/humble/setup.bash
source "/mnt/c/Users/admin/Documents/无人机强化学习 2/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /l4/nearest_distance --once
ros2 topic echo /l4/nearest_point --once
ros2 topic echo /l4/nearest_normal --once
```

### 终端 6：启动感知版 AKPF

第一轮只验证感知距离进入 AKPF，局部目标仍关闭，避免继续使用真值几何生成局部目标。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run l3_akpf_navigation l3_akpf_node --ros-args \
  -p scenario:=S1_single_front_obstacle \
  -p mission_timeout_s:=220.0 \
  -p use_stamped_cmd_vel:=true \
  -p odom_topic:=/mavros/local_position/local \
  -p pose_topic:=/mavros/local_position/pose \
  -p distance_source:=perception_map \
  -p perception_cloud_topic:=/l4/local_cloud \
  -p perception_fallback_to_truth:=false \
  -p enable_local_target:=false \
  -p max_xy_speed:=0.35 \
  -p candidate_safe_margin:=0.15 \
  -p candidate_comfort_margin:=0.50 \
  -p local_target_clearance:=0.15 \
  -p emergency_d_eff:=0.35 \
  -p recovery_exit_d_eff:=0.85 \
  -p recovery_exit_path_margin:=0.15 \
  -p recovery_min_duration_s:=3.0 \
  -p recovery_xy_speed:=0.22 \
  -p recovery_climb_speed:=0.12
```

期望看到：

```text
Distance source: perception_map
perception=yes
nearest=perception_map
```

---

## 4. 验收观察点

第一轮不要急着追求完整避障成功，先确认链路正确：

```text
1. Gazebo GUI 可见，用户能观察无人机和障碍物；
2. /l4/depth_camera/points 持续发布；
3. /mavros/state connected=true；
4. /mavros/local_position/pose 有实际消息；
5. /l4/local_cloud 持续发布，frame_id=map；
6. /l4/nearest_distance 在无人机接近障碍物时变小；
7. l3_akpf_node 日志显示 Distance source: perception_map；
8. l3_akpf_node 等待依赖时 perception=yes；
9. 不出现 Point cloud stale 或 Perception distance source is stale。
```

---

## 5. 当前边界

L4.3 当前还不是最终“感知版 AKPF 成果表”，它只是把感知距离链路接到可飞验证入口。

仍需后续验证或改进：

```text
1. 相机外参目前使用固定平移，默认无旋转；
2. 当前 local map 来自局部点云最近点，不是 ESDF；
3. 未知空间还没有安全建模；
4. 点云视野受前向相机限制，侧后方障碍不可见；
5. perception_map 模式下局部目标默认关闭，避免继续使用真值几何生成局部目标。
```

---

## 6. 下一步

完成本教程的多终端仿真验证后，建议进入：

```text
1. 多次复测 S1-S5，统计 goal_dist、min_clearance、恢复模式触发次数；
2. 评估是否把 enable_local_target 改为点云局部目标，而不是使用真值几何局部目标；
3. 进入 `L5_SafetyShield安全层复现教程.md`，把 L3 raw velocity 先接入独立 Safety Shield。
```
