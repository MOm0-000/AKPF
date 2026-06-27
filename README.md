# AKPF

AKPF 是一个基于 ArduPilot 的无人机仿真与避障研究工程。当前主线已经从真值几何 AKPF 逐步推进到 L7.1 点云感知策略部署：Gazebo 深度相机点云经 L4 局部地图进入 L7 policy，策略输出再经过 L5 Safety Shield 控制 ArduPilot SITL。

当前已完成一轮端到端验证：

```text
Gazebo depth camera
  -> L4 local point cloud/map
  -> L7.1 perception policy
  -> L5 Safety Shield
  -> MAVROS velocity setpoint
  -> ArduPilot SITL
```

S1-S5 固定场景均已 `goal_reached`。这条链路的避障输入来自 `/l4/local_cloud`、`/l4/nearest_distance`、`/l4/nearest_normal`，不是 `SCENARIOS` 中的障碍物真值几何。

## 当前技术栈

```text
Ubuntu 22.04 / WSL2
ROS2 Humble
Gazebo Harmonic / Gazebo Sim 8
ArduPilot SITL / ArduCopter
MAVROS2
```

当前项目选择 ArduPilot。

## 工程注意事项

MAVROS local 坐标不能默认把 `{0,0}` 当作起飞点。仿真中 EKF origin、模型生成点和起飞点通常接近重合，但真机/RTK 中 EKF origin 可能来自首次稳定定位时刻，不一定等于解锁起飞位置。后续真机任务应在起飞前后捕获 `mission_origin`，所有航点和高度判断使用相对起飞点坐标；ROS/MAVROS local 侧按 ENU 理解，若要相对机头飞行还需要记录起飞 yaw 并做旋转。同时必须把 GUIDED/ARM、EKF/GPS/罗盘健康问题和软件原点假设分开排查。

## 最快复现 L7.1 点云链路

在 WSL 发行版 `ld666` 中运行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
bash scripts/open_l7_1_perception_terminals.sh S5
```

脚本会依次打开 Gazebo GUI、ArduPilot SITL、MAVROS、L4 bridge、L4 mapper、L5 Shield 和 L7.1 policy。终端 2 的 SITL 命令保持为：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

任务正常降落、触发 FAILSAFE/异常或疑似坠机后，脚本默认等待 15 秒并自动清理本次仿真相关进程和终端窗口。保留窗口调试时可加：

```bash
bash scripts/open_l7_1_perception_terminals.sh S5 --no-auto-cleanup
```

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

并补充 3 个已落地进阶几何压力场景：

```text
S6_cluttered_boxes
S8_vertical_constraint
S9_multi_corner
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

### L3：真值几何 AKPF 非学习版

完成独立 ROS2 节点 `l3_akpf_navigation`，使用 L2 场景中的已知障碍物几何生成 AKPF 速度指令。当前 S1-S5 基础场景已完成验收。S1/S5 等问题场景通过通用候选速度、通用几何局部目标和终端区处理，不依赖场景专用局部目标点。

核心文件：

```text
src/l3_akpf_navigation
scripts/run_l3_akpf.sh
```

参考文档：

```text
L3_ArduPilot真值几何AKPF复现教程.md
L3_真值几何AKPF验收记录.md
```

### L3.5：进阶真值几何压力测试

完成 S6/S8/S9 三个进阶真值几何场景验收，用于在进入 L4 感知层前压测多障碍切换、低顶板高度约束和多转角局部极小。L3.5 仍然不使用点云、深度相机或强化学习，也不加入场景专用固定路点。

验收结果：

```text
S6_cluttered_boxes：goal_dist=0.48, min_clearance=0.81
S8_vertical_constraint：goal_dist=0.48, min_clearance=0.57
S9_multi_corner：goal_dist=0.48, min_clearance=0.58
```

参考文档：

```text
L3_5_进阶真值几何AKPF压力测试复现教程.md
```

### L4：点云/局部地图 AKPF

已完成 L4.3 感知版 AKPF 链路：Gazebo 深度相机点云进入 ROS2 后，由项目内 bridge 转为轻量 XYZ `PointCloud2`，`l4_pointcloud_mapper_node` 维护局部体素记忆并发布 `/l4/local_cloud`、`/l4/nearest_distance`、`/l4/nearest_point`、`/l4/nearest_normal`，L3 AKPF 可通过 `distance_source:=perception_map` 使用感知局部地图替代真值几何距离。

L4.3 已在 S1-S5 基础场景完成多终端 Gazebo + SITL + MAVROS + bridge + mapper + AKPF 验证。当前修正保持通用化：局部体素记忆、候选速度安全裕度、恢复模式迟滞和切向恢复不绑定任何单一场景。

验收结果：

```text
S1_single_front_obstacle:  GOAL pos=(4.06, 0.45, 2.03), goal_dist=0.47, min_clearance=0.79
S2_narrow_gate:           GOAL pos=(3.79, -0.28, 2.01), goal_dist=0.50, min_clearance=0.81
S3_corridor:              GOAL pos=(3.71, -0.00, 2.01), goal_dist=0.49, min_clearance=1.01
S4_table_or_low_obstacle: GOAL pos=(3.72, 0.02, 2.05), goal_dist=0.48, min_clearance=1.22
S5_corner:                GOAL pos=(3.32, 2.63, 2.00), goal_dist=0.48, min_clearance=0.75
```

核心文件：

```text
src/l4_perception_mapping
models/iris_with_l4_depth_camera
worlds/l4/S1_depth_camera.sdf
worlds/l4/S2_depth_camera.sdf
worlds/l4/S3_depth_camera.sdf
worlds/l4/S4_depth_camera.sdf
worlds/l4/S5_depth_camera.sdf
```

参考文档：

```text
L4_点云局部地图AKPF复现教程.md
L4_2_Gazebo深度相机点云桥接验证教程.md
L4_3_相机点云到MAVROS局部坐标验证教程.md
L4_点云局部地图验收记录.md
```

### L5：Safety Shield 安全层

已完成第一版独立 ROS2 节点 `l5_safety_shield`，用于接收 L3/L4 导航层 raw velocity，根据最近障碍距离、最近点法向、飞控状态和高度约束做统一安全裁剪，再发布到 MAVROS 速度 setpoint。L5 已接入 L4.3 感知版 AKPF，S1-S5 均已通过。

验收结果：

```text
S1_single_front_obstacle:  GOAL pos=(4.04, 0.45, 2.03), goal_dist=0.48, min_clearance=0.79
S2_narrow_gate:           GOAL pos=(3.79, -0.25, 2.02), goal_dist=0.48, min_clearance=0.79
S3_corridor:              GOAL pos=(3.71, 0.00, 2.01), goal_dist=0.49, min_clearance=1.01
S4_table_or_low_obstacle: GOAL pos=(3.74, 0.02, 2.04), goal_dist=0.46, min_clearance=1.21
S5_corner:                GOAL pos=(3.32, 2.63, 2.00), goal_dist=0.48, min_clearance=0.78
```

核心文件：

```text
src/l5_safety_shield
```

参考文档：

```text
L5_SafetyShield安全层复现教程.md
```

### L6：简化环境强化学习

已完成第一版轻量训练闭环，并补充 L6.1/L6.2/L6.3 工程入口：`l6_rl_training` 提供二维定高 Gym-style 环境、S1-S5 抽象几何、随机场景生成、baseline/AKPF/full/perception 观测、L5 风格速度裁剪、自包含 PyTorch PPO 入口和策略评估入口。

当前第一阶段使用 AKPF 观测、Safety Shield 和 L3 风格 teacher 行为克隆 warm-start，导出策略后在 S1-S5 简化环境中每个场景评估 10 次均通过：

```text
S1_single_front_obstacle:  success=1.00, mean_goal_dist=0.4731, mean_min_clearance=0.5688
S2_narrow_gate:           success=1.00, mean_goal_dist=0.4721, mean_min_clearance=0.6588
S3_corridor:              success=1.00, mean_goal_dist=0.4581, mean_min_clearance=1.0706
S4_table_or_low_obstacle: success=1.00, mean_goal_dist=0.4621, mean_min_clearance=1.1873
S5_corner:                success=1.00, mean_goal_dist=0.4510, mean_min_clearance=0.5994
```

截至 2026-06-27，L6.1-L6.3 的第一轮结论如下：

```text
L6.1 固定 S1-S5 对比：AKPF 观测比 baseline 更快得到可用解，但 PPO 后期仍会回落；
L6.2 fixed perception：best checkpoint 可在固定 S1-S5 评估 5/5，最终 checkpoint 会退化；
L6.3 mixed perception BC：random 30 x 5 为 145/150，失败从碰撞转为超时，安全性更好。
```

因此当前 L7.1 Gazebo perception 部署优先使用：

```text
artifacts/tmp_archive_20260627/l6_training/l6_l63_perception_mixed_bc/policy_best.pt
```

核心文件：

```text
src/l6_rl_training
```

参考文档：

```text
L6_简化环境强化学习复现教程.md
```

### L7：ROS2/Gazebo 策略部署

已完成 ROS2/Gazebo 策略部署节点 `l7_policy_deployment`，并将当前主线推进到 L7.1 perception。节点加载 L6 导出的 policy，订阅 MAVROS local odom/pose 捕获 `mission_origin`，执行策略推理，把 raw velocity 发布到 `/l5/raw_cmd_vel`，最后由 L5 Safety Shield 输出 MAVROS 速度 setpoint。

L7 目前有两种模式：

```text
geometry   旧版/对照链路，使用 SCENARIOS 抽象几何构造避障特征；
perception 当前主线，使用 L4 点云/局部地图构造避障特征。
```

当前推荐策略：

```text
artifacts/tmp_archive_20260627/l6_training/l6_l63_perception_mixed_bc/policy_best.pt
```

该 checkpoint 为 `obs_mode=perception`。实际运行日志已确认：

```text
encoder_mode=perception
/l4/local_cloud -> l7_policy_node
NAVIGATING ... map_age=...
```

因此当前 L7.1 的避障输入来自 L4 点云/局部地图，而不是障碍物真值几何。`SCENARIOS` 仍用于场景名和默认目标点，这属于任务配置；真机阶段应替换为外部任务目标接口。

验收结果：

```text
S1_single_front_obstacle:  goal_reached
S2_narrow_gate:           goal_reached
S3_corridor:              goal_reached
S4_table_or_low_obstacle: goal_reached
S5_corner:                goal_reached
```

核心文件：

```text
src/l7_policy_deployment
scripts/open_l7_1_perception_terminals.sh
```

参考文档：

```text
L7_ROS2_Gazebo策略部署复现教程.md
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

## 工程路线

总体路线见：

```text
ArduPilot_AKPF_工程实施路线.md
```

当前下一步：把已跑通的 L7.1 perception 链路做成正式统计。优先补齐 S1-S5 多轮重复实验、S6/S8/S9 感知部署复核、未知/随机障碍布局验证，以及 Shield 激活、map_age、point_count、cloud_valid、inference_ms 等指标表。固定场景和未知布局都稳定前，不进入 L8 近壁/近地扰动注入。
