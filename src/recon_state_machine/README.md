# recon_state_machine

ROS 2 Humble + MAVROS + ArduPilot 的 C++ 侦察任务状态机。

## Build

```bash
cd ~/uav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select recon_state_machine --symlink-install
source install/setup.bash
```

## Run

先启动 ArduPilot SITL/Gazebo 和 MAVROS，再运行：

```bash
ros2 launch recon_state_machine recon_state_machine.launch.py use_sim_time:=true
```

实机通常使用：

```bash
ros2 launch recon_state_machine recon_state_machine.launch.py use_sim_time:=false
```

## Required MAVROS interfaces

Topics:
- `/mavros/state`
- `/mavros/local_position/odom`
- `/mavros/setpoint_position/local`

Services:
- `/mavros/set_mode`
- `/mavros/cmd/arming`
- `/mavros/cmd/takeoff`
- `/mavros/cmd/land`
