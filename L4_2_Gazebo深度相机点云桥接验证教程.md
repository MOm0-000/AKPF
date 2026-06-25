# L4.2 Gazebo 深度相机点云桥接验证教程

日期：2026-06-14
更新：2026-06-21

本文目标：把 L4.1 中的合成 `/l4/points` 替换为 Gazebo 仿真深度相机产生的点云。完成本教程后，你可以在 Gazebo 中启动带深度相机的 Iris，将 Gazebo 点云桥接为 ROS2 `/l4/depth_camera/points`，再用现有 `l4_pointcloud_mapper_node` 输出最近距离、最近点和粗法向。

---

## 1. 当前结论

本轮已经完成：

```text
1. 新增带前向 RGBD/depth camera 的 ArduPilot Iris 模型；
2. 新增 L4 专用 Gazebo 世界；
3. 确认 Gazebo 能发布 /l4/depth_camera/points；
4. 确认 Gazebo 点云消息类型为 gz.msgs.PointCloudPacked；
5. 新增 native bridge 节点和 bridge 脚本，把 Gazebo 点云映射到 ROS2 `/l4/depth_camera/points`；
6. 新增 Gazebo 点云版 mapper 启动脚本。
```

已完成完整验证：

```text
Gazebo /l4/depth_camera/points
  -> l4_gz_pointcloud_bridge_node
  -> ROS2 /l4/depth_camera/points
  -> l4_pointcloud_mapper_node
  -> nearest distance / point / normal
```

说明：`ros_gz_bridge` 已经安装，也可以创建 bridge，但对当前 `gz.msgs.PointCloudPacked` 转换会报 `Unknown message type [8]/[9]`，ROS2 侧收不到稳定 PointCloud2。因此当前默认使用项目内 `l4_gz_pointcloud_bridge_node`，它会将 Gazebo 原始点云抽样重打包为轻量 XYZ PointCloud2。

---

## 2. 新增文件

模型：

```text
models/iris_with_l4_depth_camera/model.sdf
models/iris_with_l4_depth_camera/model.config
```

Gazebo 世界：

```text
worlds/l4/S1_depth_camera.sdf
```

脚本：

```text
scripts/install_l4_ros_gz_bridge.sh
scripts/start_l4_depth_world.sh
scripts/check_l4_gz_topics.sh
scripts/start_l4_pointcloud_bridge.sh
scripts/run_l4_gazebo_mapper.sh
src/l4_perception_mapping/src/l4_gz_pointcloud_bridge_node.cpp
```

---

## 3. 确认 bridge

进入项目目录：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
```

本机已经安装过 `ros-humble-ros-gz-bridge`。如果你换机器或重装环境，再执行：

```bash
./scripts/install_l4_ros_gz_bridge.sh
```

确认安装成功：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_bridge
```

期望有路径输出，例如：

```text
/opt/ros/humble
```

---

## 4. 编译 L4 包

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
source /opt/ros/humble/setup.bash
colcon build --packages-select l4_perception_mapping
source install/setup.bash
```

确认节点：

```bash
ros2 pkg executables l4_perception_mapping
```

期望：

```text
l4_perception_mapping l4_pointcloud_mapper_node
l4_perception_mapping l4_synthetic_cloud_node
l4_perception_mapping l4_gz_pointcloud_bridge_node
```

---

## 5. 终端 1：启动 L4 Gazebo 世界

headless 模式：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l4_depth_world.sh S1_depth_camera
```

如果要看 Gazebo 可视化：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
GZ_SERVER_ONLY=false ./scripts/start_l4_depth_world.sh S1_depth_camera
```

这个世界使用：

```text
model://iris_with_l4_depth_camera
```

相机参数：

```text
sensor type: rgbd_camera
topic: /l4/depth_camera
points topic: /l4/depth_camera/points
update_rate: 15 Hz
image: 320 x 240
clip: 0.20 m - 8.0 m
```

---

## 6. 终端 2：确认 Gazebo 点云话题

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/check_l4_gz_topics.sh
```

期望看到：

```text
/l4/depth_camera/camera_info
/l4/depth_camera/depth_image
/l4/depth_camera/image
/l4/depth_camera/points
```

确认点云消息类型：

```bash
gz topic -i -t /l4/depth_camera/points
```

本次 Codex 已验证到：

```text
Publishers [Address, Message Type]:
  ... gz.msgs.PointCloudPacked
```

---

## 7. 终端 3：启动 Gazebo -> ROS2 点云 bridge

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l4_pointcloud_bridge.sh
```

脚本默认执行：

```text
Gazebo topic: /l4/depth_camera/points
ROS2 topic:   /l4/depth_camera/points
bridge:       l4_gz_pointcloud_bridge_node
output:       sampled XYZ sensor_msgs/msg/PointCloud2
```

默认参数：

```text
BRIDGE_IMPL=native
REPACK_XYZ=true
SAMPLE_STEP=4
```

注意：当前 Humble 版 `ros_gz_bridge parameter_bridge` 可用 `BRIDGE_IMPL=ros_gz` 切回，但本环境下它对 `PointCloudPacked` 转换不稳定。L4.2 验收以 native bridge 为准。

如果你的 Gazebo 点云 topic 名不同，可以这样覆盖：

```bash
POINT_CLOUD_TOPIC=/your/gz/points ./scripts/start_l4_pointcloud_bridge.sh
```

如果想保留 ROS2 topic 名不变但覆盖输出 topic：

```bash
ROS_POINT_CLOUD_TOPIC=/l4/points ./scripts/start_l4_pointcloud_bridge.sh
```

另开一个终端检查 ROS2 话题：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t | grep /l4
```

期望看到：

```text
/l4/depth_camera/points [sensor_msgs/msg/PointCloud2]
```

---

## 8. 终端 4：运行 L4 mapper

Gazebo 深度相机点云通常在相机坐标系中，不是 MAVROS local ENU 坐标系。因此当前 L4.2 验证阶段不要用 odom 查询点，先使用传感器坐标系原点：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/run_l4_gazebo_mapper.sh
```

脚本默认参数：

```text
POINTCLOUD_TOPIC=/l4/depth_camera/points
USE_ODOM=false
QUERY_X=0.0
QUERY_Y=0.0
QUERY_Z=0.0
VOXEL_SIZE_M=0.15
LOCAL_RADIUS_M=6.0
```

期望看到类似：

```text
L4 pointcloud mapper started: cloud=/l4/depth_camera/points ... use_odom=false
L4 query pos=(0.00, 0.00, 0.00) input=4800 local=545 nearest=0.78 point=(0.73, -0.00, -0.25)
```

如果无人机在初始位置，前方障碍物在相机前方，`nearest` 应该是一个正距离，通常在几米以内。具体数值取决于相机坐标系、点云采样、障碍表面和起飞/未起飞状态。

---

## 9. 可选：接入 SITL/MAVROS 后飞行验证

如果你要让无人机飞起来，同时观察点云距离随运动变化，启动顺序是：

```text
终端 1：./scripts/start_l4_depth_world.sh S1_depth_camera
终端 2：ArduPilot SITL
终端 3：./scripts/start_mavros_l1.sh
终端 4：./scripts/start_l4_pointcloud_bridge.sh
终端 5：./scripts/run_l4_gazebo_mapper.sh
终端 6：L1 或 L3 状态机
```

SITL 推荐命令：

```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild --out=udp:127.0.0.1:14550
```

MAVROS：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

注意：此时 mapper 仍建议 `USE_ODOM=false`。因为当前还没有做相机点云 frame 到 MAVROS local ENU 的 TF 变换。要真正替换 L3 真值几何，需要先完成 L4.3 的坐标变换和感知距离接口。

---

## 10. 当前边界和下一步

本教程验证的是：

```text
Gazebo rgbd_camera
  -> /l4/depth_camera/points
  -> l4_gz_pointcloud_bridge_node
  -> /l4/depth_camera/points
  -> l4_pointcloud_mapper_node
  -> nearest distance / point / normal
```

还不能直接声称完成“感知版 AKPF 飞行避障”，因为缺少：

```text
1. 相机点云坐标系到 MAVROS local ENU 的 TF/外参变换；
2. 点云视野外和未知空间处理；
3. L3 AKPF 的距离查询接口替换；
4. 飞行过程中的时延和噪声评估。
```

下一步 L4.3：

```text
1. 固定相机外参；
2. 将点云变换到 local ENU；
3. 让 mapper 输出 local-frame nearest obstacle；
4. 修改 L3 AKPF，使其可以选择 truth_geometry 或 perception_map 两种距离来源；
5. 先在 S1_depth_camera 中验证感知距离随无人机接近障碍物而减小。
```

详细步骤见：

```text
L4_3_相机点云到MAVROS局部坐标验证教程.md
```
