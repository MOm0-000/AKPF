# L3.5 进阶真值几何 AKPF 压力测试复现教程

日期：2026-06-14

适用环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Harmonic / Gazebo Sim 8
ArduPilot SITL / ArduCopter
MAVROS2
```

本文目标：在 L3 真值几何 AKPF 已经通过 S1-S5 基础场景后，继续使用同一套非学习版 AKPF 节点，对更复杂的几何场景做压力测试。L3.5 仍然不使用点云、不使用深度相机、不使用强化学习；它的作用是先把“真值几何 + 速度控制 + 通用局部目标”的上限和弱点跑清楚，再进入 L4 感知层。

---

## 1. L3.5 做什么

L3.5 的输入仍然是：

```text
L2/L3 场景名称
场景内已知 box 障碍物几何
/mavros/local_position/odom
/mavros/state
目标点
```

L3.5 的输出仍然是：

```text
/mavros/setpoint_velocity/cmd_vel
```

L3.5 新增验证重点：

```text
S6_cluttered_boxes：多障碍交错，检查局部目标切换与净空保持；
S8_vertical_constraint：低顶板/高度约束，检查高度控制和近障碍风险；
S9_multi_corner：多转角，检查局部极小和主动脱困能力。
```

暂不纳入本轮 L3.5 的场景：

```text
S7_table_chair_room：更适合在 L4 有点云/局部地图后做，因为桌椅类小结构需要感知输入；
S10_perception_degradation：名字本身就是感知退化，应放到 L4/L5 后验证，不适合真值几何阶段。
```

---

## 2. 重要边界

L3.5 不是规划器，也不是强化学习策略。它只验证 AKPF 在已知几何下能否稳定产生可执行速度。

本层不做：

```text
不接深度相机；
不建点云地图；
不做 ESDF；
不训练 RL；
不做场景专用硬编码路线；
不为某个场景写固定中间路点。
```

当前局部目标是通用几何局部目标：

```text
1. 如果当前位置到全局目标的直线净空足够，则直接追踪全局目标；
2. 如果直线被膨胀障碍物挡住，则从障碍物膨胀角点自动选择可见局部目标；
3. 局部目标只由障碍物几何、安全半径和线段净空判断生成；
4. 局部目标未到达且仍能推进全局目标时，会保持一段时间，避免频繁切换；
5. 当局部目标方向的预测净空安全时，优先沿 AKPF 场方向前进，避免候选速度评分把飞机推回原路。
```

这仍然属于真值几何阶段，因为障碍物列表来自场景定义。真实应用中，这个列表会在 L4 被点云/局部地图替换。

---

## 3. 涉及文件

项目根目录：

```text
/mnt/c/Users/admin/Documents/无人机强化学习 2
```

核心节点：

```text
src/l3_akpf_navigation/src/l3_akpf_node.cpp
```

运行脚本：

```text
scripts/run_l3_akpf.sh
scripts/start_l2_world.sh
scripts/start_mavros_l1.sh
```

场景协议：

```text
protocols/l2_scenarios.yaml
```

进阶场景：

```text
worlds/l2/S6_cluttered_boxes.sdf
worlds/l2/S8_vertical_constraint.sdf
worlds/l2/S9_multi_corner.sdf
```

---

## 4. 编译

进入项目目录：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
```

加载 ROS2 环境：

```bash
source /opt/ros/humble/setup.bash
```

编译 L3 AKPF 包：

```bash
colcon build --packages-select l3_akpf_navigation
```

期望看到：

```text
Finished <<< l3_akpf_navigation
Summary: 1 package finished
```

本次验收实际结果：

```text
Summary: 1 package finished [24.9s]
```

---

## 5. 推荐终端布局

每个场景建议打开 4 个终端：

```text
终端 1：Gazebo 场景
终端 2：ArduPilot SITL
终端 3：MAVROS
终端 4：L3 AKPF 状态机
```

每跑完一个场景，建议全部关闭后再跑下一个场景，避免端口和旧进程残留。

---

## 6. 终端 1：启动 Gazebo 场景

以 S6 为例：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh S6_cluttered_boxes
```

如果要启动 S8：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh S8_vertical_constraint
```

如果要启动 S9：

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_l2_world.sh S9_multi_corner
```

说明：

```text
默认脚本按 headless/server 模式运行 Gazebo，适合稳定验收。
如果需要可视化界面，使用：
GZ_SERVER_ONLY=false ./scripts/start_l2_world.sh S6_cluttered_boxes
```

---

## 7. 终端 2：启动 ArduPilot SITL

进入 ArduPilot 目录：

```bash
cd ~/ardupilot
```

推荐使用轻量启动命令：

```bash
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild --out=udp:127.0.0.1:14550
```

看到以下内容即可继续：

```text
Detected vehicle 1:1 on link 0
Received 1424 parameters
ArduPilot Ready
```

注意：

```text
如果使用 --map --console 后出现 EOF on TCP socket、MAVProxy 断连或窗口异常，
本层验收优先使用上面的轻量命令，不打开 map/console。
```

---

## 8. 终端 3：启动 MAVROS

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
./scripts/start_mavros_l1.sh
```

看到以下内容即可继续：

```text
Plugin setpoint_velocity initialized
CON: Got HEARTBEAT, connected. FCU: ArduPilot
```

如果长时间没有心跳：

```text
1. 确认 SITL 已经 ArduPilot Ready；
2. 确认 SITL 使用 --out=udp:127.0.0.1:14550；
3. 关闭 MAVROS 后重新启动；
4. 必要时按 Gazebo -> SITL -> MAVROS 的顺序全量重启。
```

---

## 9. 终端 4：运行 S6

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
SCENARIO=S6_cluttered_boxes MISSION_TIMEOUT_S=150 ./scripts/run_l3_akpf.sh
```

期望流程：

```text
Dependencies ready
FCU connected
GUIDED already active
Vehicle armed
Takeoff reached ... starting AKPF navigation
Goal reached
State -> HOVER
State -> LANDING
State -> DONE
```

本次验收关键结果：

```text
Goal reached: pos=(4.01, 0.51, 2.00), goal_dist=0.48, min_clearance=0.81
```

结论：S6 通过。

---

## 10. 终端 4：运行 S8

S8 的目标高度是 1.55 m，主要测试低顶板约束下的高度与水平避障。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
SCENARIO=S8_vertical_constraint MISSION_TIMEOUT_S=140 ./scripts/run_l3_akpf.sh
```

本次验收关键结果：

```text
Scenario: S8_vertical_constraint, goal=(4.20, 0.00, 1.55), obstacles=7
Takeoff reached 1.85 m, starting AKPF navigation
Goal reached: pos=(3.78, -0.21, 1.46), goal_dist=0.48, min_clearance=0.57
State -> HOVER
State -> LANDING
Landed or disarmed, mission complete
State -> DONE
```

结论：S8 通过。

---

## 11. 终端 4：运行 S9

S9 是多转角场景，主要测试局部目标保持、停滞检测和局部极小脱困。

```bash
cd "/mnt/c/Users/admin/Documents/无人机强化学习 2"
SCENARIO=S9_multi_corner MISSION_TIMEOUT_S=160 ./scripts/run_l3_akpf.sh
```

本次验收关键结果：

```text
Goal reached: pos=(4.21, 2.22, 2.00), goal_dist=0.48, min_clearance=0.58
```

结论：S9 通过。

---

## 12. 本次 L3.5 通用修正

L3.5 过程中重点修正了两个泛化问题。

### 12.1 局部目标保持

之前的逻辑会要求“当前位置到当前局部目标的整条线段一直保持可见”。在多障碍、多转角场景中，无人机绕过障碍后，局部目标可能会被刚绕过的障碍短暂遮挡，导致局部目标被过早丢弃，状态机又回到全局目标或重新选点，形成徘徊。

当前逻辑改为：

```text
只要当前局部目标尚未到达；
并且该局部目标仍然比当前位置更接近全局目标；
并且局部目标点本身有净空；
就继续保持该局部目标。
```

这不是场景专用路点，而是所有场景共用的局部目标生命周期规则。

### 12.2 安全局部目标方向优先

候选速度评分会同时考虑进度、平滑性和预测净空。这个机制在贴近障碍时有用，但在局部目标已经选好、AKPF 场方向本身安全时，过强的净空奖励可能把飞机推向“更远离障碍但偏离目标”的方向。

当前逻辑改为：

```text
如果正在使用局部目标；
并且 AKPF 场方向的预测净空不为负；
则优先沿 AKPF 场方向前进；
否则再进入候选速度评分。
```

这样可以减少“看起来更安全但实际退回原路”的振荡。

---

## 13. S9 场景可行性说明

S9 曾经出现过一个不是算法问题、而是场景几何过约束的问题：目标点被边界墙和内部墙体压得太紧，目标半径 0.5 m 内几乎没有足够安全净空，导致无人机即使接近目标也会被多面墙体斥力推出。

当前 S9 做的是场景可行性修正：

```text
目标点调整为 [4.40, 2.65, 2.00]；
第三段转角墙体缩短为 size [0.18, 0.95, 2.70]；
墙体中心调整为 [3.45, 2.85, 1.35]。
```

这不是给 S9 写路线，也不是给 S9 添加固定中间路点；它只是保证目标区域本身满足“目标半径内存在可安全到达区域”。

---

## 14. 验收汇总

| 场景 | 目标 | 结果 | 关键结果 |
| --- | --- | --- | --- |
| S6_cluttered_boxes | 多障碍交错 | 通过 | `goal_dist=0.48, min_clearance=0.81` |
| S8_vertical_constraint | 低顶板/高度约束 | 通过 | `goal_dist=0.48, min_clearance=0.57` |
| S9_multi_corner | 多转角/局部极小 | 通过 | `goal_dist=0.48, min_clearance=0.58` |

L3.5 当前结论：

```text
真值几何 AKPF 已能通过基础场景 S1-S5；
并通过进阶几何压力场景 S6、S8、S9；
S7 和 S10 应推迟到 L4/L5，因为它们依赖感知输入和感知退化建模。
```

---

## 15. 清理进程

推荐按顺序停止：

```text
终端 4：L3 AKPF，正常会自行退出；
终端 3：MAVROS，按 Ctrl-C；
终端 2：ArduPilot SITL，按 Ctrl-C；
终端 1：Gazebo，按 Ctrl-C。
```

检查残留：

```bash
pgrep -af 'mavros_node|start_mavros_l1|sim_vehicle.py|arducopter|gz sim|gzserver'
```

如果没有输出，表示清理完成。

---

## 16. 下一步

L3.5 结束后，不建议继续在真值几何里堆更多手工场景。下一步进入 L4：

```text
L4.1：Gazebo 深度相机/点云接入；
L4.2：点云裁剪、降采样和最近点距离查询；
L4.3：用点云距离替换 L3 的真值几何距离；
L4.4：在 S1-S6/S8/S9 中对比真值几何和感知几何的成功率、最小净空和到达时间。
```

L4 成功后，再回到 S7_table_chair_room 和 S10_perception_degradation 会更合理。
