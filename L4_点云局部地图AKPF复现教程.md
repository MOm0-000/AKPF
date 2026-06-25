# L4 点云/局部地图 AKPF 复现教程

日期：2026-06-14

适用环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Harmonic / Gazebo Sim 8
ArduPilot SITL / ArduCopter
MAVROS2
```

本文目标：从 L3/L3.5 的“真值几何障碍物列表”过渡到 L4 的“点云/局部地图查询”。本阶段先完成 L4.1：ROS2 `PointCloud2` 输入、局部裁剪、voxel 降采样、最近点距离和粗法向估计。

当前 L4.1 使用合成点云发布器做离线验收，把 PointCloud2 处理链路先跑通。L4.2 已安装 `ros_gz_bridge`，并准备了 Gazebo 深度相机 world 和 bridge 脚本，用于把 Gazebo `/l4/depth_camera/points` 桥接为 ROS2 同名 PointCloud2。

---

## 1. L4.1 做什么

输入：

```text
/l4/points                     sensor_msgs/msg/PointCloud2
/mavros/local_position/odom     nav_msgs/msg/Odometry，可选
```

输出：

```text
/l4/local_cloud         sensor_msgs/msg/PointCloud2
/l4/nearest_distance    std_msgs/msg/Float32
/l4/nearest_point       geometry_msgs/msg/PointStamped
/l4/nearest_normal      geometry_msgs/msg/Vector3Stamped
```

处理流程：

```text
PointCloud2
  -> 读取 x/y/z 字段
  -> 按无人机当前位置或指定查询点做局部半径裁剪
  -> voxel 降采样
  -> 最近点查询
  -> 粗法向估计 normal = normalize(query_position - nearest_point)
  -> 发布距离、最近点、法向和局部点云
```

---

## 2. 本层边界

L4.1 做：

```text
ROS2 PointCloud2 接口；
局部点云裁剪；
voxel 降采样；
最近点距离查询；
粗法向估计；
合成点云离线验收。
```

L4.1 暂不做：

```text
不接真实深度相机；
不接 ros_gz_bridge；
不替换 L3 飞行闭环中的真值几何；
不做占据栅格；
不做 ESDF；
不训练 RL。
```

合成点云发布器只用于验证 PointCloud2 处理链路，不是最终科研方案中的感知来源。

---

## 3. 新增文件

项目根目录：

```text
/mnt/c/Users/admin/Documents/无人机强化学习 2
```

新增 ROS2 包：

```text
src/l4_perception_mapping
```

核心节点：

```text
src/l4_perception_mapping/src/l4_pointcloud_mapper_node.cpp
src/l4_perception_mapping/src/l4_synthetic_cloud_node.cpp
```

运行脚本：

```text
scripts/run_l4_mapping_demo.sh
```

验收记录：

```text
L4_点云局部地图验收记录.md
```

---

## 4. 编译 L4 包

进入项目目录：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
```

加载 ROS2：

```bash
source /opt/ros/humble/setup.bash
```

编译 L4 包：

```bash
colcon build --packages-select l4_perception_mapping
```

期望看到：

```text
Finished <<< l4_perception_mapping
Summary: 1 package finished
```

本次实测：

```text
Finished <<< l4_perception_mapping [21.3s]
Summary: 1 package finished [21.6s]
```

确认节点存在：

```bash
source install/setup.bash
ros2 pkg executables l4_perception_mapping
```

期望输出：

```text
l4_perception_mapping l4_pointcloud_mapper_node
l4_perception_mapping l4_synthetic_cloud_node
```

---

## 5. 一键离线 demo

执行：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l4_mapping_demo.sh
```

脚本默认做以下事情：

```text
1. 启动 l4_synthetic_cloud_node；
2. 发布 S1_single_front_obstacle 的合成 PointCloud2 到 /l4/points；
3. 使用查询点 query=(2.00, 0.00, 2.00)；
4. 启动 l4_pointcloud_mapper_node；
5. 运行约 8 秒后自动退出；
6. 清理后台合成点云发布器。
```

期望看到：

```text
L4 synthetic cloud started: scenario=S1_single_front_obstacle boxes=5 points=9078
L4 pointcloud mapper started: cloud=/l4/points ... use_odom=false
L4 query pos=(2.00, 0.00, 2.00) input=9078 local=5022 nearest=0.66 ...
```

本次实测关键输出：

```text
L4 query pos=(2.00, 0.00, 2.00) input=9078 local=5022 nearest=0.66 point=(2.65, -0.09, 1.96) normal=(-0.99, 0.14, 0.05)
```

说明：

```text
S1 的 front_block 中心在 x=3.00，尺寸 x=0.70；
靠近无人机一侧表面约在 x=2.65；
查询点 x=2.00；
因此最近距离约 0.65 m，实测 0.66 m 合理。
```

---

## 6. 换场景运行

S6 多障碍：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
SCENARIO=S6_cluttered_boxes QUERY_X=2.40 QUERY_Y=-0.70 QUERY_Z=2.00 ./scripts/run_l4_mapping_demo.sh
```

S8 低顶板：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
SCENARIO=S8_vertical_constraint QUERY_X=2.65 QUERY_Y=0.00 QUERY_Z=1.55 ./scripts/run_l4_mapping_demo.sh
```

S9 多转角：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
SCENARIO=S9_multi_corner QUERY_X=2.60 QUERY_Y=0.30 QUERY_Z=2.00 ./scripts/run_l4_mapping_demo.sh
```

可调参数：

```bash
DURATION_S=12
VOXEL_SIZE_M=0.12
LOCAL_RADIUS_M=5.0
SAMPLE_STEP_M=0.15
QUERY_X=2.0
QUERY_Y=0.0
QUERY_Z=2.0
```

示例：

```bash
DURATION_S=12 VOXEL_SIZE_M=0.12 ./scripts/run_l4_mapping_demo.sh
```

---

## 7. 手动双终端运行

终端 1：合成点云发布器。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run l4_perception_mapping l4_synthetic_cloud_node --ros-args \
  -p scenario:=S1_single_front_obstacle \
  -p sample_step_m:=0.18
```

终端 2：局部地图 mapper。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run l4_perception_mapping l4_pointcloud_mapper_node --ros-args \
  -p use_odom:=false \
  -p query_x:=2.00 \
  -p query_y:=0.00 \
  -p query_z:=2.00
```

查看输出话题：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t | grep /l4
```

期望包括：

```text
/l4/local_cloud [sensor_msgs/msg/PointCloud2]
/l4/nearest_distance [std_msgs/msg/Float32]
/l4/nearest_normal [geometry_msgs/msg/Vector3Stamped]
/l4/nearest_point [geometry_msgs/msg/PointStamped]
/l4/points [sensor_msgs/msg/PointCloud2]
```

查看最近距离：

```bash
ros2 topic echo /l4/nearest_distance --once
```

---

## 8. 接入 MAVROS 位姿

如果已经启动 ArduPilot、MAVROS，并且 `/mavros/local_position/odom` 正常发布，可以让 mapper 使用无人机当前位置作为查询点：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run l4_perception_mapping l4_pointcloud_mapper_node --ros-args \
  -p use_odom:=true \
  -p odom_topic:=/mavros/local_position/odom
```

此时不需要设置 `query_x/query_y/query_z`，mapper 会用 odom 里的当前位置。

注意：当前点云来源仍是合成 `/l4/points`。下一步需要把 `/l4/points` 换成 Gazebo 深度相机/点云 bridge 输出。

---

## 9. 当前环境中的 bridge 状态

本次检查 ROS2 包时，发现当前环境存在：

```text
sensor_msgs
pcl_conversions
pcl_msgs
tf2_ros
tf2_sensor_msgs
```

但没有发现：

```text
ros_gz_bridge
```

因此 L4.2 的建议顺序是：

```text
1. 安装或确认 ros_gz_bridge；
2. 在 iris 或独立传感器模型中加入深度相机/点云传感器；
3. 用 ros_gz_bridge 把 Gazebo 点云桥接到 /l4/points；
4. 保持 l4_pointcloud_mapper_node 不变；
5. 对比合成点云和 Gazebo 点云的最近距离稳定性。
```

---

## 10. 常见问题

### 10.1 一键 demo 一开始显示 Waiting for PointCloud2

WSL 下 FastDDS 发现可能较慢。脚本已经加入：

```text
ros2 node list warmup
ros2 topic list warmup
ros2 topic echo /l4/points --once warmup
```

如果仍然偶发等待，可以把时间加长：

```bash
DURATION_S=20 ./scripts/run_l4_mapping_demo.sh
```

### 10.2 nearest 距离和预期差一点

这是正常的。当前合成点云是按表面网格采样，再进行 voxel 降采样，最近点不一定正好落在几何解析最近点上。

可以减小采样和 voxel：

```bash
SAMPLE_STEP_M=0.10 VOXEL_SIZE_M=0.08 ./scripts/run_l4_mapping_demo.sh
```

### 10.3 为什么不用 KD-tree

当前 L4.1 先用 voxel 降采样后的线性最近点扫描，原因是点数较小、依赖少、便于验证接口。后续当接入真实点云后，如果点数和频率压力变大，再替换为 PCL KD-tree 或 nanoflann。

### 10.4 这是不是又在用真值几何

合成点云发布器确实是从场景几何采样得到的，它只用于 L4.1 的离线单元验收。L4 真正要替换的是 `/l4/points` 的来源：从合成点云换成 Gazebo 深度相机/真实传感器点云。

---

## 11. 本层完成标准

L4.1 当前完成标准：

```text
l4_perception_mapping 可以编译；
l4_synthetic_cloud_node 可以发布 PointCloud2；
l4_pointcloud_mapper_node 可以订阅 PointCloud2；
可以完成局部裁剪和 voxel 降采样；
可以输出最近距离、最近点、粗法向；
一键 demo 能稳定复现。
```

下一步是 L4.2：

```text
Gazebo 深度相机/点云传感器
  -> ros_gz_bridge
  -> /l4/points
  -> l4_pointcloud_mapper_node
  -> L3 AKPF 感知距离替换
```


---

## 12. L4.2 Gazebo 深度相机桥接

L4.1 合成点云链路通过后，下一份教程是：

```text
L4_2_Gazebo深度相机点云桥接验证教程.md
```

该教程使用 `models/iris_with_l4_depth_camera` 和 `worlds/l4/S1_depth_camera.sdf`，把 Gazebo 的 `/l4/depth_camera/points` 通过 `ros_gz_bridge` 桥接到 ROS2 `/l4/points`。
