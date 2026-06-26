# ArduPilot 版 AKPF 工程实施路线

本文档只描述工程实施路线，不包含代码实现。

目标是把 AKPF 科研方案从“论文级整体框架”拆成一层层可验证、可回退、可发表的工程步骤。路线遵循一个原则：

> 先确保 ArduPilot 飞行闭环稳定，再逐层增加避障能力、风险场、感知、强化学习和扰动鲁棒性。

当前主线环境：

- Ubuntu 22.04.5 LTS
- 用户 `ld666`
- ROS2 Humble
- Gazebo Harmonic
- ArduPilot Copter SITL
- `ardupilot_gazebo`
- MAVROS
- 已有 `~/ws_offboard` 状态机验证工程

边界：

- 选择 ArduPilot，放弃 PX4；
- 使用 ArduPilot `GUIDED` 模式，不使用 PX4 `OFFBOARD` 模式；
- `COD` conda 环境只用于解析文件，不用于运行代码、训练或仿真；
- 本文档只描述工程路线和验收边界，具体实现与复现命令见各层级文档、`src/` 和 `scripts/`。

---

## 0. 总体分层路线

工程路线分为 9 层：

```text
L0  固化现有基线
L1  ArduPilot 速度控制闭环
L2  室内仿真场景与实验协议
L3  真值几何 AKPF 非学习版
L3.5 进阶真值几何压力测试
L4  点云/局部地图 AKPF
L5  Safety Shield 安全层
L6  简化环境强化学习
L7  ROS2/Gazebo 策略部署
L8  近壁/近地扰动注入
L9  系统实验、消融与论文固化
```

核心思想：

- L0-L2 是平台层；
- L3-L5 是可解释避障层；
- L6-L7 是学习层；
- L8 是鲁棒性创新层；
- L9 是论文闭环层。

不要跳层。每一层没有通过验收，就不要进入下一层。

当前工程进度：

```text
L0-L2 已完成；
L3 真值几何 AKPF 已完成，S1-S5 已完成验收；
L3.5 进阶真值几何压力测试已完成，S6/S8/S9 已通过；
L4.3 感知版 AKPF 已完成，S1-S5 已通过；
L5 Safety Shield 初版已接入 L4.3 感知链路，S1-S5 已通过；
L6 简化环境强化学习第一版已完成，AKPF+Shield warm-start 策略在简化 S1-S5 中已通过；
L7 ROS2 策略部署节点第一版已完成，已通过不启动 Gazebo 的 ROS2 合成冒烟，并已完成 Gazebo S1-S5 一轮部署验证；
S7_table_chair_room 和 S10_perception_degradation 延后到 L6/L7 前后；
下一步继续 L6 PPO 长训、baseline 对比、训练曲线统计，并在 L7 中补充多次重复实验和 Shield 激活统计。
```

---

## L0：固化现有基线

### 目标

把当前已经能跑的 Gazebo + ArduPilot + MAVROS + 状态机流程固定下来，作为后续所有修改的回退基线。

### 为什么先做这一层

当前环境已经能完成起飞、矩形航线、降落，这是宝贵的最低可用闭环。后续任何改动如果导致飞不起来，都必须能回到这个基线判断问题来自哪里。

### 输入

- `~/ardupilot`
- `~/ardupilot_gazebo`
- `~/ws_offboard`
- `offboard_control offb_node`
- 当前 `iris_runway.sdf`

### 输出

- 一份可重复运行的基线记录；
- 一组标准启动命令；
- 一份基线日志；
- 一份常见问题表。

### 应记录内容

记录以下信息：

- Gazebo 启动命令；
- ArduPilot SITL 启动命令；
- MAVROS 启动命令；
- 状态机启动命令；
- MAVROS 连接成功标志；
- `GUIDED` 成功标志；
- 解锁成功标志；
- 起飞高度；
- 矩形航线实际轨迹；
- 降落是否正常；
- 每次运行是否出现 mode/arm 不稳定。

### 验收标准

至少连续 3 次完成：

```text
Gazebo 启动
  -> ArduPilot SITL 连接
  -> MAVROS 连接
  -> GUIDED
  -> arm
  -> takeoff
  -> 矩形航线
  -> land
```

如果 3 次里有 1 次失败，要记录失败阶段。

### 风险

- Word 文档中的长横线导致命令错误；
- conda base 自动激活污染 ROS/Gazebo 环境；
- `GUIDED` 或 arm 偶发失败；
- Gazebo NVIDIA 渲染异常。

### 本层不做

- 不改状态机逻辑；
- 不引入避障；
- 不引入 RL；
- 不改 MAVROS 参数；
- 不写新包。

---

## L1：ArduPilot 速度控制闭环

### 目标

把当前“位置 setpoint 状态机”升级为“速度 setpoint 控制闭环”，为后续 AKPF 和 RL 输出速度指令做准备。

### 为什么必须做

AKPF 和 RL 更适合输出：

```text
[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]
```

如果继续用位置 setpoint，避障模块每一步都要临时构造目标点，Safety Shield 也更难做速度裁剪。

### 输入

- L0 的稳定基线；
- MAVROS 当前可用话题；
- `/mavros/state`
- `/mavros/local_position/odom`
- MAVROS setpoint velocity 相关接口。

### 输出

一个速度控制验证流程，能完成：

- 起飞到指定高度；
- 前进；
- 后退；
- 左移；
- 右移；
- 上升；
- 下降；
- 悬停；
- 降落。

### 推荐状态机

```text
WAITING_FCU
  -> SETTING_GUIDED
  -> ARMING
  -> TAKEOFF
  -> VELOCITY_TEST
  -> HOVER
  -> LANDING
  -> DONE
```

### 关键问题

需要确认：

- MAVROS 当前暴露哪个速度控制话题；
- 使用 `TwistStamped` 还是 `Twist`；
- MAVROS 是否要求持续发布 setpoint；
- 速度坐标系是 body frame 还是 local ENU；
- yaw rate 接口是否可用；
- 发布频率低于多少会失效。

### 推荐控制约束

第一阶段速度要保守：

```text
v_xy_max = 0.5 m/s
v_z_max = 0.3 m/s
yaw_rate_max = 0.3 rad/s
publish_rate >= 20 Hz
```

调通后再逐步提高。

### 验收标准

必须满足：

- `GUIDED` 模式稳定保持；
- 解锁后不会立刻重新上锁；
- 速度指令能明显改变无人机运动；
- 悬停阶段漂移可接受；
- 指令停止后不会继续危险飞行；
- 可以安全降落；
- 连续 3 次测试成功。

### 风险

- MAVROS 速度话题坐标系理解错误；
- 发布频率不足；
- ArduPilot 接收速度指令但行为滞后；
- yaw rate 不生效；
- 高度控制和水平速度控制互相干扰。

### 本层不做

- 不做避障；
- 不做地图；
- 不做 RL；
- 不做复杂轨迹。

---

## L2：室内仿真场景与实验协议

### 目标

从 `iris_runway.sdf` 过渡到室内避障场景，建立后续所有算法的标准测试环境。

### 为什么这一层在 AKPF 前面

没有标准场景，就无法判断 AKPF 是否有效。先做场景和评价协议，后面每一层都能用同一套场景横向比较。

### 输入

- Gazebo Harmonic；
- `ardupilot_gazebo` 模型；
- 当前 `iris_runway.sdf`；
- AKPF 论文设定中的室内场景类型。

### 输出

最小场景集：

```text
S1_single_front_obstacle
S2_narrow_gate
S3_corridor
S4_table_or_low_obstacle
S5_corner
```

中期场景集：

```text
S6_cluttered_boxes
S7_table_chair_room
S8_vertical_constraint
S9_multi_corner
S10_perception_degradation
```

### 每个场景必须定义

- 起点；
- 目标点；
- 安全飞行高度范围；
- 障碍物尺寸；
- 任务最大时间；
- 成功条件；
- 碰撞条件；
- 超时条件；
- 轨迹记录格式。

### 推荐成功条件

```text
distance_to_goal < 0.5 m
and no_collision
and flight_time < T_max
```

### 推荐失败条件

```text
collision == true
or altitude < min_altitude
or altitude > max_altitude
or distance_to_start becomes unreasonable
or timeout
or FCU disconnected
```

### 评价指标

从这一层开始固定指标：

- Success Rate；
- Collision Rate；
- Time to Goal；
- Path Length；
- Minimum Clearance；
- Mean Speed；
- Max Speed；
- Control Jitter；
- Mode Drop Count；
- Shield Activation Count，后续 L5 启用。

### 验收标准

至少完成：

- 5 个基础场景能正常打开；
- 无人机能在每个场景起飞；
- 场景不会卡 Gazebo；
- 起点和目标点明确；
- 人工或基线状态机可以跑通至少一个简单任务。

### 风险

- 场景太复杂导致仿真卡顿；
- 障碍物碰撞体与视觉模型不一致；
- 室内空间太小导致 ArduPilot 起飞阶段就出问题；
- 目标点设置不合理。

### 本层不做

- 不要求自动随机生成大规模场景；
- 不要求深度相机；
- 不要求自动 reset；
- 不要求 RL。

---

## L3：真值几何 AKPF 非学习版

### 目标

在不引入深度相机、不引入 ESDF、不引入 RL 的情况下，先用已知障碍物几何验证 AKPF 的核心数学项。

### 为什么要用真值几何

AKPF 本身已经包含多个变量：

- 目标吸引；
- 动力学有效距离；
- 气动代理风险；
- 旋度偏置；
- 速度裁剪。

如果一开始就叠加点云、地图和 RL，出问题时无法判断是地图错、控制错还是风险场错。

### 输入

- L1 速度控制；
- L2 室内场景；
- 场景中障碍物的已知几何；
- 无人机位置和速度；
- 目标点。

### 输出

一个非学习版 AKPF 局部导航器：

```text
obstacle geometry
  -> distance and normal query
  -> AKPF field
  -> desired velocity
  -> MAVROS velocity command
```

### 最小 AKPF 版本

第一版只实现：

```text
U_att + U_kino
```

第二版加入：

```text
F_curl
```

第三版加入：

```text
U_aero
```

不要一开始把所有项全打开。

### 推荐子阶段

#### L3.1：目标吸引

只让无人机朝目标飞，不考虑障碍。

验收：

- 能飞向目标；
- 能在目标附近减速；
- 不出现大幅振荡。

#### L3.2：几何排斥

加入基础障碍排斥。

验收：

- 单障碍能绕开；
- 不撞障碍；
- 控制指令平滑。

#### L3.3：动力学有效距离

加入：

```text
d_eff = d - d_brake - d_delay - r_body - margin
```

验收：

- 速度越快，越早避障；
- 高速正前障碍中比静态距离版本更安全；
- 最小 `d_eff` 不小于阈值。

#### L3.4：旋度偏置

加入正前障碍和窄门对称破缺。

验收：

- 正前障碍不长时间悬停；
- 窄门入口左右摆动减少；
- `Control Jitter` 下降。

#### L3.5：气动代理风险与进阶几何压力测试

加入近墙、近地、近桌面风险项，但先只影响引导或惩罚，不做扰动。随后用更复杂的真值几何场景做压力测试，确认通用局部目标和候选速度选择不是只在简单场景中偶然通过。

验收：

- 靠墙飞行时保持更大距离；
- 贴地/贴桌面飞行减少；
- 不造成过度保守；
- 在 S6/S8/S9 中验证局部目标切换、高度约束和多转角脱困。

### 验收标准

至少在 S1-S5 中完成：

- S1 单障碍成功；
- S2 窄门成功；
- S3 走廊不撞墙；
- S4 不贴桌面；
- S5 墙角不陷入局部极小。

### 当前 L3 状态

截至 2026-06-14，L3 已完成真值几何 AKPF 节点，并通过：

```text
S1_single_front_obstacle
S2_narrow_gate
S3_corridor
S4_table_or_low_obstacle
S5_corner
```

当前 L3 的修正原则是通用化处理：房间边界墙统一纳入障碍物距离查询，AKPF 输出后用局部候选速度评分选择方向；当直达目标的线段被膨胀障碍物挡住时，自动从障碍物膨胀角点生成几何局部目标；进入目标终端区后提高目标方向权重、适度软化斥力。没有使用 S1 或 S5 专用局部目标点或固定绕行路线。

S1-S5 已经作为 L3 基础场景关闭。后续不再继续在 L3 中堆更多真值几何特例，而是用 L3.5 做进入 L4 前的进阶压力测试。

### 风险

- 势场参数过强导致飞不动；
- 吸引和排斥互相抵消；
- 旋度方向抖动；
- `d_eff` 接近 0 时数值爆炸；
- 速度指令过大导致 ArduPilot 响应滞后。

### 本层不做

- 不做点云；
- 不做 ESDF；
- 不做 RL；
- 不做气动扰动注入；
- 不追求最优路径。

---


## L3.5：进阶真值几何压力测试

### 目标

在进入点云/局部地图之前，用更复杂但仍可控的真值几何场景压测 AKPF，确认当前非学习版算法不是只在 S1-S5 的简单几何中偶然通过。

### 为什么不直接做 S7/S10

S7_table_chair_room 更接近桌椅、小物体和局部遮挡问题，S10_perception_degradation 本身就是感知退化问题。它们应该在 L4 有点云/局部地图输入、L5 有安全层之后再做，否则仍然只是把真实问题手工写进真值几何列表。

### 已验收场景

```text
S6_cluttered_boxes
S8_vertical_constraint
S9_multi_corner
```

### 本层验证重点

- S6：多障碍交错，检查局部目标切换和净空保持；
- S8：低顶板/高度约束，检查高度控制和近障碍风险；
- S9：多转角，检查局部极小、停滞检测和局部目标保持。

### 通用修正

L3.5 中继续坚持不写场景专用路线。当前采用的泛化修正是：

```text
1. 保持仍能推进全局目标的局部目标，避免频繁丢弃；
2. 当局部目标方向本身预测安全时，优先沿 AKPF 场方向前进；
3. 只有在局部场方向不安全时，才进入候选速度评分；
4. 进度停滞时强制重新生成局部目标。
```

### 当前结果

```text
S6_cluttered_boxes：goal_dist=0.48, min_clearance=0.81，通过；
S8_vertical_constraint：goal_dist=0.48, min_clearance=0.57，通过；
S9_multi_corner：goal_dist=0.48, min_clearance=0.58，通过。
```

详细命令与日志见：

```text
L3_5_进阶真值几何AKPF压力测试复现教程.md
```

### 本层结论

L3/L3.5 已经足够支撑进入 L4。后续真实应用中的关键问题不再是继续扩写真值障碍物列表，而是把障碍物距离、法向和局部可通行空间从传感器/地图中估计出来。

---

## L4：点云/局部地图 AKPF

### 目标

把 L3 中的真值障碍距离替换为传感器或局部地图估计，使 AKPF 从“知道场景几何”过渡到“依赖感知”。

### 输入

- L3 的 AKPF 非学习版；
- Gazebo 深度相机或点云；
- 无人机位姿；
- 局部窗口参数。

### 输出

局部地图查询能力：

```text
point cloud / depth
  -> local obstacle representation
  -> distance query
  -> normal query
  -> AKPF samples
```

### 推荐三步走

#### L4.1：点云最近点查询

最简单可行方案：

- 点云裁剪；
- voxel 降采样；
- KD-tree 最近点；
- 最近点方向近似法向。

优点：

- 工程量较小；
- 能快速替换真值距离；
- 适合 AKPF 采样点查询。

缺点：

- 法向粗糙；
- 障碍背面和遮挡处理弱；
- 不是真正 ESDF。

#### L4.2：局部占据栅格

构建局部 voxel grid：

- 只保留无人机周围局部窗口；
- 维护占据/未知/自由；
- 可加入简单膨胀。

优点：

- 能处理局部空间；
- 更接近后续 ESDF。

#### L4.3：局部 ESDF

参考：

- FIESTA 的增量 ESDF；
- voxblox 的 TSDF/ESDF；
- Fast-Planner 的地图模块。

但这一阶段不要直接整仓库迁移，优先实现项目需要的最小查询能力。

### 地图频率建议

```text
点云输入：10-30 Hz
局部地图更新：5-10 Hz
AKPF 查询：20-50 Hz
控制输出：50 Hz 左右
```

### 验收标准

- 点云距离估计与真值距离误差可接受；
- 法向方向基本稳定；
- AKPF 查询频率满足 20 Hz 以上；
- S1-S5 成功率接近真值版本；
- 感知延迟不会引发明显撞障。

### 风险

- 深度相机话题和坐标系配置复杂；
- 点云噪声导致法向抖动；
- 地图未知区域处理不当；
- 地图更新慢；
- 坐标变换 TF 错误。

### 本层不做

- 不训练 RL；
- 不做大规模随机地图；
- 不做复杂语义理解；
- 不追求全局建图。

### 当前 L4 状态

截至 2026-06-14，L4.1 已完成最小点云局部地图链路：

```text
PointCloud2
  -> local crop
  -> voxel downsample
  -> nearest point query
  -> distance and rough normal
```

新增产物：

```text
src/l4_perception_mapping
scripts/run_l4_mapping_demo.sh
L4_点云局部地图AKPF复现教程.md
L4_点云局部地图验收记录.md
```

当前验收结果：

```text
S1 合成点云：input=9078, local=5022, nearest=0.66 m，通过。
```

注意：截至 2026-06-25，L4.3 已完成 Gazebo 深度相机点云到 MAVROS local ENU 的完整链路验证。Gazebo 能发布 `/l4/depth_camera/points`，项目内 `l4_gz_pointcloud_bridge_node` 将 `gz.msgs.PointCloudPacked` 抽样重打包为轻量 XYZ `PointCloud2`；`l4_pointcloud_mapper_node` 维护 90 秒局部体素记忆并发布 `/l4/local_cloud`、`/l4/nearest_distance`、`/l4/nearest_point`、`/l4/nearest_normal`；L3 AKPF 支持 `truth_geometry` / `perception_map` 两种距离来源。按 `L4_3_相机点云到MAVROS局部坐标验证教程.md` 复现，S1-S5 均已到达目标，并已支撑 L5 Safety Shield 初版接入验证。

---

## L5：Safety Shield 安全层

### 目标

建立独立安全层，对 AKPF 或 RL 输出的速度指令进行裁剪、降速、悬停或降落保护。

### 为什么必须独立

Safety Shield 不能依赖策略是否聪明。无论上层是：

- 手写 AKPF；
- APF；
- RL；
- MPC；
- 人工遥控；

最终速度指令都必须经过统一安全层。

### 输入

- 原始速度指令；
- `d_eff_min`；
- 前向距离；
- 当前速度；
- 地图质量；
- MAVROS 状态；
- 高度；
- 最近一次感知时间；
- 最近一次控制时间。

### 输出

- 安全速度指令；
- shield 状态；
- shield 激活原因；
- 日志。

### 推荐规则

#### 前向速度限制

```text
v_forward_max = k * max(0, d_front - d_safe)
```

#### 动力学距离保护

```text
if d_eff_min < d_stop:
    forbid forward motion
    allow lateral/backward escape
```

#### 地图失效保护

```text
if map_timeout:
    hover
```

#### FCU 异常保护

```text
if not connected:
    stop publishing navigation commands

if mode != GUIDED:
    try recover
    if recover failed:
        land or hover
```

#### 高度保护

```text
if z < z_min or z > z_max:
    override vz_cmd
```

### 验收标准

人工输入危险速度时：

- 能自动限速；
- 不撞正前障碍；
- 地图超时时悬停；
- MAVROS 状态异常时不继续乱飞；
- 所有 shield 激活都有日志。

### 风险

- 规则过强导致飞不动；
- 规则过弱保护不了；
- 地图质量指标设计粗糙；
- 悬停和避障指令冲突。

### 本层不做

- 不训练 RL；
- 不做复杂形式化验证；
- 不承诺实机绝对安全。

---

## L6：简化环境强化学习

### 目标

在轻量环境中训练基于 AKPF 特征的 RL 策略，验证 AKPF 表示是否比原始距离/深度/占据图更容易学习。

### 为什么不直接用 Gazebo 训练

Gazebo + ArduPilot SITL：

- reset 慢；
- 并行困难；
- 训练样本效率低；
- 容易被仿真稳定性拖住。

因此主训练应在简化环境中进行，Gazebo 用于部署验证。

### 输入

- L3/L4 的 AKPF 特征定义；
- L2 场景抽象；
- 无人机简化动力学；
- 障碍物几何。

### 输出

- Gym 风格训练环境设计；
- SAC/PPO 策略；
- AKPF 观测和 baseline 观测对比；
- 训练曲线；
- 策略导出格式。

### 简化动力学建议

第一版：

```text
p_{t+1} = p_t + v_cmd * dt
v_{t+1} = low_pass(v_t, v_cmd)
```

第二版：

```text
加入最大加速度
加入速度延迟
加入 yaw 约束
```

第三版：

```text
加入扰动和观测噪声
```

### 观测对比

至少比较：

1. 原始距离采样；
2. 原始点云降维；
3. AKPF without `U_kino`；
4. AKPF full features；
5. AKPF + `F_curl`；
6. AKPF + Safety Shield。

### 奖励从简单开始

第一版：

```text
r = progress + goal_bonus - collision_penalty - timeout_penalty - action_smooth_penalty
```

第二版：

```text
加入 d_eff 惩罚
```

第三版：

```text
加入 AKPF 对齐奖励
加入 aero 风险惩罚
```

### 验收标准

- 策略在简化环境中能稳定收敛；
- AKPF 观测收敛速度优于 baseline；
- AKPF 成功率高于 baseline；
- AKPF 最小 `d_eff` 更大；
- 输出动作没有严重抖动。

### 风险

- 简化动力学和 ArduPilot 差异过大；
- 奖励设计让策略学会奇怪行为；
- AKPF 特征过强，RL 贡献不明显；
- baseline 设计不公平。

### 本层不做

- 不使用 COD；
- 不直接依赖 Gazebo 训练；
- 不做实机；
- 不做气动插件。

---

## L7：ROS2/Gazebo 策略部署

### 目标

把 L6 训练出的策略部署到 ROS2 中，通过 MAVROS 控制 ArduPilot SITL，在 Gazebo 室内场景中验证。

### 输入

- 训练好的策略；
- L4 局部地图或 L3 真值 AKPF；
- L5 Safety Shield；
- L1 速度控制桥。

### 输出

策略部署链路：

```text
sensor/map
  -> AKPF feature encoder
  -> policy inference
  -> Safety Shield
  -> MAVROS velocity command
```

### 部署频率建议

```text
feature encoding: 20-50 Hz
policy inference: 20-50 Hz
shield: 20-50 Hz
velocity publish: 50 Hz
```

### 关键工程问题

- 策略输入归一化必须和训练一致；
- 动作范围必须和训练一致；
- 推理延迟要记录；
- 感知缺失时不能给策略喂假稳定数据；
- 策略输出必须经过 Safety Shield；
- 每次 episode 要记录日志。

### 验收标准

在 S1-S5 场景中：

- 策略能完成至少 80% 简单任务；
- 无明显高频震荡；
- Safety Shield 激活次数可解释；
- 推理频率满足要求；
- 失败案例可复现。

### 风险

- sim-to-sim gap：简化环境策略到 Gazebo 失效；
- 观测归一化错误；
- 坐标系错误；
- 策略输出速度过激；
- ArduPilot 响应延迟导致训练假设不成立。

### 本层不做

- 不做气动扰动；
- 不追求论文最终成绩；
- 不上实机。

### 当前 L7 状态

截至 2026-06-26，L7 第一版已完成 ROS2/Gazebo 部署链路验证。`l7_policy_node` 加载 L6 导出的 `/tmp/l6_akpf_bc_pass/policy.pt`，使用 MAVROS local odom/pose 捕获 `mission_origin`，按相对起飞点坐标编码 AKPF 观测，在线推理后发布 raw velocity 到 `/l5/raw_cmd_vel`，再由 L5 Safety Shield 裁剪并输出 MAVROS 速度 setpoint。

Gazebo S1-S5 一轮部署验证结果：

```text
S1_single_front_obstacle:  GOAL pos=(4.29, 0.45, 2.00), goal_dist=0.46
S2_narrow_gate:           GOAL pos=(3.73, 0.14, 2.00), goal_dist=0.49
S3_corridor:              GOAL pos=(3.75, 0.05, 2.00), goal_dist=0.46
S4_table_or_low_obstacle: GOAL pos=(3.74, -0.08, 2.00), goal_dist=0.47
S5_corner:                GOAL pos=(3.35, 2.56, 2.00), goal_dist=0.46
```

每个场景验证结束后均清理 Gazebo、SITL、MAVProxy、MAVROS、bridge、mapper、shield、policy node 相关进程，并确认残留计数为 0。

---

## L8：近壁/近地扰动注入

### 目标

加入低成本状态依赖扰动，验证 AKPF 的气动风险代理是否能提升近墙、近地、近桌面飞行鲁棒性。

### 何时进入这一层

必须满足：

- L3 AKPF 非学习版可用；
- L5 Safety Shield 可用；
- L7 策略可部署；
- 实验日志完善；
- 至少有近墙/近地场景。

### 输入

- Gazebo 场景；
- 无人机位姿；
- 最近墙面/地面/桌面距离；
- 机体姿态；
- 当前速度；
- AKPF `U_aero`。

### 输出

扰动实验能力：

```text
near wall / near ground state
  -> surrogate force and torque
  -> Gazebo applies disturbance
  -> controller/policy responds
```

### 扰动分级

至少设置：

```text
none
weak
medium
strong
unseen_random
```

### 对比实验

比较：

- raw policy；
- AKPF without `U_aero`；
- AKPF with `U_aero`；
- AKPF with `U_aero` + disturbance training；
- AKPF full。

### 验收标准

在扰动场景中：

- AKPF full 成功率最高或退化最小；
- 姿态方差降低；
- 控制抖动降低；
- 近墙/近地碰撞率降低；
- `U_aero` 消融后性能明显下降。

### 风险

- 扰动模型过于启发式；
- 扰动过强导致所有方法都失败；
- 扰动过弱看不出差异；
- 评审质疑气动真实性。

### 表述边界

论文中必须明确：

- 这不是 CFD；
- 这是 surrogate disturbance；
- 目标是鲁棒性训练和验证；
- 贡献是风险代理与扰动注入一致性，而不是精确流体仿真。

### 本层不做

- 不追求高保真空气动力学；
- 不做复杂 CFD；
- 不把扰动模型当真实物理结论。

---

## L9：系统实验、消融与论文固化

### 目标

形成论文可用的完整实验闭环。

### 输入

- L1-L8 全部模块；
- 场景集；
- baseline；
- 日志；
- 训练曲线；
- profiling 数据。

### 输出

论文核心结果：

- 主结果表；
- 消融实验表；
- 鲁棒性实验表；
- 对称冲突专项表；
- 计算开销表；
- 轨迹图；
- 速度/姿态曲线；
- Safety Shield 激活图；
- 失败案例分析。

### 主结果表

比较：

- APF；
- AKPF without RL；
- raw distance RL；
- AKPF RL；
- AKPF RL + Safety Shield；
- AKPF full。

场景：

- 高速正前障碍；
- 窄门；
- 走廊；
- 近墙；
- 近地/近桌面；
- 角隅；
- 随机杂乱。

### 消融表

必须有：

- full；
- w/o `U_kino`；
- w/o `U_aero`；
- w/o `F_curl`；
- w/o Safety Shield；
- w/o disturbance training；
- `F_curl` replaced by discrete left/right choice。

### 计算开销表

记录：

- 点云预处理耗时；
- 地图更新耗时；
- AKPF 查询耗时；
- 策略推理耗时；
- Safety Shield 耗时；
- MAVROS 指令发布延迟；
- 总闭环延迟。

### 失败案例分析

至少分析：

- 对称窄门失败；
- 角隅局部极小；
- 感知缺失；
- ArduPilot 响应延迟；
- Safety Shield 过度保守；
- 扰动过强。

### 验收标准

可以支撑论文的标准：

- 每个实验有固定随机种子或可复现实验配置；
- 每个结果不是单次运行；
- 每个指标定义清晰；
- 每个 baseline 公平；
- 每个消融只改一个变量；
- 所有失败案例有解释。

---

## 20. 最小可发表路线

如果时间有限，建议压缩为：

```text
L0 -> L1 -> L2 -> L3 -> L5 -> L9
```

也就是：

1. 固化 ArduPilot 基线；
2. 做速度控制；
3. 做室内场景；
4. 做 AKPF 非学习版；
5. 做 Safety Shield；
6. 做系统实验和消融。

这个路线可以形成一篇偏“物理先验风险场 + ArduPilot 工程验证”的论文或开题成果。

---

## 21. 完整科研路线

如果时间充足，完整路线为：

```text
L0 -> L1 -> L2 -> L3 -> L4 -> L5 -> L6 -> L7 -> L8 -> L9
```

这对应一篇更完整的“AKPF 引导强化学习 + ArduPilot 部署 + 近场扰动鲁棒性”的论文。

---

## 22. 推荐时间安排

### 第 1 月

- 完成 L0；
- 完成 L1；
- 开始 L2 基础场景。

### 第 2 月

- 完成 L2；
- 完成 L3.1-L3.3；
- 得到 AKPF 基础避障结果。

### 第 3 月

- 完成 L3.4-L3.5；
- 完成 L5；
- 开始系统性 baseline 对比。

### 第 4 月

- 完成 L4 点云/局部地图第一版；
- 替换真值距离；
- 对比真值 AKPF 与感知 AKPF。

### 第 5-6 月

- 完成 L6 简化环境 RL；
- 完成 AKPF 观测和 baseline 观测对比。

### 第 7 月

- 完成 L7 策略部署；
- 在 Gazebo/ArduPilot 中跑 RL policy。

### 第 8 月

- 完成 L8 扰动注入第一版；
- 做近墙/近地鲁棒性实验。

### 第 9-10 月

- 扩充场景；
- 做消融；
- 做 profiling。

### 第 11-12 月

- 整理论文图表；
- 写论文；
- 整理代码和实验说明。

---

## 23. 每层完成后的决策点

每层结束后回答三个问题：

1. 这一层是否稳定可复现？
2. 进入下一层会不会掩盖当前层的问题？
3. 当前层是否已经产生可写进论文的结果？

如果任意一个答案是否定的，就不要急着进入下一层。

---

## 24. 最重要的工程纪律

1. 不跳过速度控制验证；
2. 不把 RL 放在第一位；
3. 不把 ESDF 放在第一位；
4. 不把气动扰动放在第一位；
5. 每层都保留 baseline；
6. 每层都记录日志；
7. 每层都定义验收标准；
8. 每个新增模块都能单独关闭；
9. 每个实验都能复现；
10. ArduPilot 是唯一飞控主线。

---

## 25. 当前下一步

当前最合理的下一步不是写 AKPF，也不是训练 RL，而是：

```text
L0：把现有基线固化成可复现运行记录
```

完成 L0 后，再进入：

```text
L1：验证 MAVROS velocity setpoint 是否能稳定控制 ArduPilot
```

这两层完成后，AKPF 才有可靠的工程落点。
