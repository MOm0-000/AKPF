# L5 Safety Shield 安全层复现教程

日期：2026-06-25

本文目标：在 L4.3 感知版 AKPF 已能完成 S1-S5 的基础上，增加一个独立 Safety Shield 节点，对上层导航输出的速度指令做统一安全裁剪，再发布到 MAVROS。

Safety Shield 不依赖 AKPF 是否聪明；后续 RL、MPC 或人工输入速度也应走同一层。

---

## 1. 当前结论

已完成：

```text
1. 新增 l5_safety_shield 包；
2. l5_safety_shield_node 订阅 /l5/raw_cmd_vel；
3. shield 订阅 /l4/nearest_distance、/l4/nearest_normal、/mavros/state、/mavros/local_position/local；
4. shield 发布裁剪后的 /mavros/setpoint_velocity/cmd_vel；
5. shield 发布 /l5/shield_status 和 /l5/shield_active；
6. l3_akpf_node 增加 stamped_cmd_vel_topic / unstamped_cmd_vel_topic 参数，默认仍保持原 MAVROS 输出。
```

已完成 L5 接入 L4.3 感知版 AKPF 的 S1-S5 多终端仿真验证。每个场景验证结束后均清理 Gazebo、SITL、MAVROS、bridge、mapper、shield、AKPF 相关进程，并确认残留计数为 0。

```text
S1_single_front_obstacle:  GOAL pos=(4.04, 0.45, 2.03), goal_dist=0.48, min_clearance=0.79
S2_narrow_gate:           GOAL pos=(3.79, -0.25, 2.02), goal_dist=0.48, min_clearance=0.79
S3_corridor:              GOAL pos=(3.71, 0.00, 2.01), goal_dist=0.49, min_clearance=1.01
S4_table_or_low_obstacle: GOAL pos=(3.74, 0.02, 2.04), goal_dist=0.46, min_clearance=1.21
S5_corner:                GOAL pos=(3.32, 2.63, 2.00), goal_dist=0.48, min_clearance=0.78
```

本轮修复保持通用性：感知 recovery 退出不再只依赖直达目标路径完全清空；当当前有效距离已经明显离开近障风险区，允许交还给正常 AKPF 候选速度层处理。这避免窄门、墙角等需要绕行或穿越可通行口的场景被 recovery 长时间锁住。

已完成合成话题自检：

```text
raw cmd: vx=0.35，nearest=0.60，normal=(-1,0,0)
shield: near_obstacle,limit_toward_obstacle
d_eff=0.1
```

---

## 2. 编译

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-select l3_akpf_navigation l4_perception_mapping l5_safety_shield --symlink-install
source install/setup.bash
```

期望包含：

```bash
ros2 pkg executables l5_safety_shield
```

```text
l5_safety_shield l5_safety_shield_node
```

---

## 3. 接入 L4.3 多终端链路

终端 1-5 仍按 `L4_3_相机点云到MAVROS局部坐标验证教程.md` 启动：

```text
终端 1：Gazebo GUI，不能加 -s
终端 2：ArduPilot SITL，使用用户指定 sim_vehicle.py 命令
终端 3：MAVROS
终端 4：Gazebo 点云 bridge
终端 5：local-frame mapper
```

### 终端 6A：启动 Safety Shield

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

期望看到：

```text
L5 safety shield started: raw=/l5/raw_cmd_vel safe=/mavros/setpoint_velocity/cmd_vel
```

### 终端 6B：启动经过 Shield 的感知版 AKPF

区别是 L3 不再直接发 MAVROS 速度话题，而是发到 `/l5/raw_cmd_vel`。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run l3_akpf_navigation l3_akpf_node --ros-args \
  -p scenario:=S1_single_front_obstacle \
  -p mission_timeout_s:=220.0 \
  -p use_stamped_cmd_vel:=true \
  -p stamped_cmd_vel_topic:=/l5/raw_cmd_vel \
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
  -p recovery_clear_exit_d_eff:=1.70 \
  -p recovery_min_duration_s:=3.0 \
  -p recovery_xy_speed:=0.22 \
  -p recovery_climb_speed:=0.12
```

S2-S5 只替换 `scenario` 和 Gazebo world：

```text
S2_narrow_gate              worlds/l4/S2_depth_camera.sdf
S3_corridor                 worlds/l4/S3_depth_camera.sdf
S4_table_or_low_obstacle    worlds/l4/S4_depth_camera.sdf
S5_corner                   worlds/l4/S5_depth_camera.sdf
```

---

## 4. 观察点

临时检查：

```bash
source /opt/ros/humble/setup.bash
source "/mnt/c/Users/admin/Documents/无人机强化学习 2/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic echo /l5/shield_status --once
ros2 topic echo /l5/shield_active --once
ros2 topic echo /l5/raw_cmd_vel --once --qos-reliability best_effort
ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once --qos-reliability best_effort
```

正常情况下可能看到：

```text
pass d_eff=...
near_obstacle,limit_toward_obstacle d_eff=...
map_timeout d_eff=...
cmd_timeout d_eff=...
not_guided d_eff=...
```

---

## 5. 验收标准

第一阶段验收：

```text
1. L5 节点能启动；
2. L3 可把 raw velocity 发到 /l5/raw_cmd_vel；
3. L5 可把 safe velocity 发到 /mavros/setpoint_velocity/cmd_vel；
4. 人工或合成危险速度会触发 near_obstacle / limit_toward_obstacle；
5. 地图超时时输出 hover，并记录 map_timeout；
6. FCU 未连接时不继续发布导航速度；
7. 所有触发都有 /l5/shield_status 日志。
```

第二阶段已把 L5 接入 S1-S5 仿真链路，当前记录：

```text
S1_single_front_obstacle:  goal_dist=0.48, min_clearance=0.79
S2_narrow_gate:           goal_dist=0.48, min_clearance=0.79
S3_corridor:              goal_dist=0.49, min_clearance=1.01
S4_table_or_low_obstacle: goal_dist=0.46, min_clearance=1.21
S5_corner:                goal_dist=0.48, min_clearance=0.78
```

后续若需要形成论文统计表，再补充多次重复实验的 `shield_active_count`、`near_obstacle_count`、`map_timeout_count` 和 `mission_time`。

---

## 6. 当前边界

```text
1. 当前 Shield 是规则层，不是形式化验证；
2. 当前使用最近点距离/法向，不是完整 ESDF；
3. 当前只处理 TwistStamped 主链路；
4. 地图质量指标还很粗糙，只使用 nearest 话题超时；
5. 当前 S1-S5 结果是单轮多终端验证，还不是多随机种子/多次重复统计。
```
