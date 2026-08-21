# cyx_arm

六自由度串联机械臂的 ROS 2 控制工程骨架。机械模型由 SolidWorks 设计，并通过 sw2urdf 导出。

本仓库基于 ROS 2 Humble、Gazebo Classic、`ros2_control` 和 MoveIt 2，当前已经具备模型显示、mock 控制、Gazebo + RViz 联合仿真以及 MoveIt 2 运动规划与轨迹执行能力。本阶段不包含达妙电机硬件接口或实机测试。

## 机械臂概况

- 构型：六自由度串联机械臂，Piper 类构型
- 关节：6 个旋转关节
- Joint 1～3 计划使用达妙 DM4340
- Joint 4～6 计划使用达妙 DM4310
- 仿真与真实硬件应使用同一套上层 ROS 2 控制接口

原始 URDF 中的关节名为 `arm_joint1`～`arm_joint6`，不是早期设计文档中的 `joint1`～`joint6`。实现控制器、SRDF 和硬件配置时必须统一命名，避免直接混用。

## 当前完成状态

已完成：

- 创建 ROS 2 workspace 风格的 `src/` 目录
- 补齐 8 个功能包的 `package.xml` 和 `CMakeLists.txt`
- 迁移 sw2urdf 导出的 URDF、CSV、STL、配置、launch 和导出日志
- 将工作 URDF 的 mesh URI 从 `package://cyx_arm_001/...` 调整为 `package://cyx_robot_description/...`
- 保留 sw2urdf 原始导出辅助文件，方便后续追溯
- 将工作模型转换为可复用的 Xacro，六个关节均配置为带位置限制的 `revolute`
- 将电机转速由 rpm 换算为 rad/s，并同步位置、峰值速度和峰值力矩到 URDF 与 `ros2_control`
- 保留关节额定/峰值参数及仿真阻尼估计到 `cyx_robot_control/config/joint_parameters.yaml`
- 实现 `robot_state_publisher`、关节状态 GUI 和 RViz 模型显示
- 配置 position command 及 position/velocity state interface
- 配置 `joint_state_broadcaster` 和 `joint_trajectory_controller`
- 实现 mock hardware 控制测试入口
- 实现 Gazebo Classic world、模型生成、控制器加载及 RViz 联合启动
- 配置 Gazebo 模型搜索路径，确保安装空间中的 STL mesh 可被正确加载
- 将机械关节硬限制集中到 `joint_limits.xacro`，由 URDF 与 `ros2_control` 共同引用
- 增加空的 `tool0` 规划坐标系，相对 `arm_link6` 沿 +Z 偏移 16 mm
- 配置 MoveIt 2 的 SRDF、KDL、OMPL、关节运行限制和轨迹控制器映射
- 实现 Gazebo、ros2_control、MoveIt 2 与 RViz MotionPlanning 联合启动
- 增加统一的程序化规划 Action、Service、运动状态机和轨迹缓存
- 实现仅含 `home` 的命名姿态、关节目标、末端位姿和笛卡尔多路点规划
- 实现 `W/S/A/D/I/K` 控制 `tool0` 每次约 10 mm 的安全笛卡尔微动
- 增加基础仿真、MoveIt、键盘控制、调试和可达空间分析的一键启动入口
- 使用 11 个基本几何体替代约 45.7 万个 STL 三角面进行运行时碰撞检测
- 完成固定种子的 SRDF 自碰撞矩阵采样，并保留所有观测到碰撞的非相邻 link 对
- 增加随机姿态自碰撞统计、CSV 报告和严格阈值检查工具
- 增加 `tool0` 可达空间、关节限位和可操作度的 RViz 可视化及 CSV 导出

尚未完成：

- 真实工作台、工件及环境障碍物的 PlanningScene 配置
- 达妙电机 `hardware_interface` 实现
- 真机通信、标定与安全测试

`cyx_robot_hardware` 当前仅保留可构建的占位包，不属于本阶段实现范围。

## 目录结构

```text
cyx_arm/
├── README.md
├── rviz/
│   └── cyx.rviz
└── src/
    ├── cyx_robot_description/   # URDF/Xacro、mesh、模型显示
    ├── cyx_robot_bringup/       # 系统统一启动入口
    ├── cyx_robot_control/       # ros2_control 控制器配置
    ├── cyx_robot_interfaces/    # 程序化运动 Action、Service 与消息
    ├── cyx_robot_planning/      # MoveIt 规划服务器、状态机与键盘微动
    ├── cyx_robot_hardware/      # 达妙电机硬件抽象层
    ├── cyx_robot_moveit/        # MoveIt 2 配置与启动
    └── cyx_robot_simulation/    # Gazebo 仿真、world 与启动文件
```

### 模型文件

| 内容 | 路径 |
| --- | --- |
| 当前工作 URDF | `src/cyx_robot_description/urdf/cyx_robot.urdf` |
| sw2urdf CSV | `src/cyx_robot_description/urdf/cyx_arm_001.csv` |
| 7 个 STL mesh | `src/cyx_robot_description/meshes/` |
| 当前工作 Xacro | `src/cyx_robot_description/urdf/cyx_robot.urdf.xacro` |
| 简化碰撞几何 | `src/cyx_robot_description/urdf/collision_geometry.xacro` |
| 机械限制参数源 | `src/cyx_robot_description/urdf/joint_limits.xacro` |
| 空 TCP 坐标系 | `src/cyx_robot_description/urdf/tool0.xacro` |
| 关节额定/峰值参数 | `src/cyx_robot_control/config/joint_parameters.yaml` |
| sw2urdf 原始辅助文件 | `src/cyx_robot_description/sw2urdf_export/` |

`sw2urdf_export/` 中的 package、CMake 和 launch 文件由 ROS 1 风格的导出器生成，仅作为原始资料保留，不应直接用于 ROS 2 构建。

运行时 Xacro 的 visual 仍使用原始 SolidWorks STL，collision 则使用 7 个圆柱体和 4 个盒体组成的简化包络。其尺寸来自 STL 在各 link 坐标系中的包围范围，用于提高 FCL/MoveIt 碰撞检测效率；这些几何是工程近似值，机械结构修改后应在 RViz 中打开 `Collision Enabled` 重新检查包络。

### 关节限制

所有内部参数均使用 SI 单位：位置为 rad、速度为 rad/s、力矩为 N·m。

| 关节 | 位置范围 | 峰值速度 | 峰值力矩 | 额定速度 | 额定力矩 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `arm_joint1` | `[-π, π]` | 5.864306 rad/s | 40 N·m | 3.769911 rad/s | 14 N·m |
| `arm_joint2` | `[-π/2, π/2]` | 5.864306 rad/s | 40 N·m | 3.769911 rad/s | 14 N·m |
| `arm_joint3` | `[-4π/5, 4π/5]` | 5.864306 rad/s | 40 N·m | 3.769911 rad/s | 14 N·m |
| `arm_joint4` | `[-π, π]` | 20.943951 rad/s | 7 N·m | 12.566371 rad/s | 3 N·m |
| `arm_joint5` | `[-π/2, π/2]` | 20.943951 rad/s | 7 N·m | 12.566371 rad/s | 3 N·m |
| `arm_joint6` | `[-π, π]` | 20.943951 rad/s | 7 N·m | 12.566371 rad/s | 3 N·m |

URDF 硬限制采用峰值速度和峰值力矩。额定值保存在 `joint_parameters.yaml`，用于后续连续运行限制及控制器调参。当前阻尼和摩擦是 Gazebo 数值稳定性所需的仿真估计值，并非电机实测值。

`joint_limits.xacro` 是 URDF 和 `ros2_control` 的机械硬限制单一参数源。MoveIt 的连续运行速度与估算加速度另存于 `cyx_robot_moveit/config/joint_limits.yaml`：速度使用额定值；1～3 轴加速度暂估为 2 rad/s²，4～6 轴暂估为 4 rad/s²。

### 末端规划坐标系

当前没有实际末端执行器。规划链使用无质量、无 visual、无 collision 的空 `tool0`：

```text
arm_link6
└── arm_link6_to_tool0 (fixed, xyz: 0 0 0.016 m)
    └── tool0
```

MoveIt planning group 为 `base_link → tool0`，轨迹控制器仍只控制 `arm_joint1`～`arm_joint6`。

## 在开发服务器上开始工作

目标环境：

- Ubuntu 22.04
- ROS 2 Humble
- `ament_cmake`
- C++17

建议安装的主要组件：

- ROS 2 Humble Desktop
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `ros2_control` 与 `ros2_controllers`
- Gazebo 与 `gazebo_ros2_control`
- `colcon`

克隆仓库：

```bash
git clone https://github.com/2159612024/cyx_arm.git
cd cyx_arm
```

安装依赖并构建：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

建议在非 Conda 环境中构建，确认 `python3` 指向 Ubuntu 22.04 的 `/usr/bin/python3`。

## 使用方法

### 一键启动入口

`cyx_robot_bringup` 提供以下入口，内部复用仿真和 MoveIt launch，不会重复创建机器人或控制器：

| 命令 | 用途 |
| --- | --- |
| `ros2 launch cyx_robot_bringup simulation.launch.py` | Gazebo、ros2_control 和可选 RViz |
| `ros2 launch cyx_robot_bringup moveit_sim.launch.py` | 完整 MoveIt 联合仿真和运动服务器 |
| `ros2 launch cyx_robot_bringup keyboard_control.launch.py` | 完整联合仿真并在当前终端读取键盘微动 |
| `ros2 launch cyx_robot_bringup workspace_analysis.launch.py` | 采样并显示可达空间 |
| `ros2 launch cyx_robot_bringup debug.launch.py` | 默认关闭 Gazebo GUI 的完整调试栈 |

常用参数为 `use_rviz`、`use_gazebo_gui` 和 `world`；MoveIt 入口另支持 `use_motion_server`。例如无 GUI 启动：

```bash
ros2 launch cyx_robot_bringup moveit_sim.launch.py \
  use_rviz:=false use_gazebo_gui:=false
```

### RViz 模型显示

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cyx_robot_description display.launch.py
```

通过 Joint State Publisher GUI 调整六个关节，RViz 会同步显示模型和 TF。

### mock ros2_control 测试

```bash
ros2 launch cyx_robot_control control.launch.py
```

该入口不启动 Gazebo，使用 `mock_components/GenericSystem` 验证控制器配置。

### Gazebo 与 RViz 联合仿真

```bash
ros2 launch cyx_robot_simulation gazebo.launch.py
```

也可以使用统一入口：

```bash
ros2 launch cyx_robot_bringup bringup.launch.py
```

无图形界面运行 Gazebo：

```bash
ros2 launch cyx_robot_simulation gazebo.launch.py \
  use_gazebo_gui:=false use_rviz:=false
```

启动后可确认控制器状态：

```bash
ros2 control list_controllers
```

预期 `joint_state_broadcaster` 和 `joint_trajectory_controller` 均为 `active`。

若 Gazebo 中实体存在但模型不可见，可检查 mesh 加载日志：

```bash
grep -R "Failed to find mesh" ~/.gazebo/server-*/default.log
```

联合启动文件会将 `cyx_robot_description` 的父级 `share/` 目录加入 `GAZEBO_MODEL_PATH`，用于解析 `model://cyx_robot_description/meshes/*.STL`。修改模型或重新构建后，必须重新执行 `source install/setup.bash`。

### MoveIt 2、Gazebo 与 RViz 联合仿真

```bash
ros2 launch cyx_robot_moveit demo.launch.py
```

也可以通过统一入口启动：

```bash
ros2 launch cyx_robot_bringup bringup.launch.py use_moveit:=true
```

在 RViz 的 MotionPlanning 显示中选择规划组 `cyx_arm`，拖动 `tool0` 的交互标记，然后使用 **Plan** 或 **Plan & Execute**。MoveIt 通过以下 Action 将六轴轨迹交给现有控制器：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

当前没有配置 3D 相机，启动时关于 Octomap 或 `/recognize_objects` 不可用的提示不影响关节空间、末端位姿或笛卡尔路径规划。

#### MoveIt 轨迹规划与执行逻辑

联合启动后的数据流如下：

```text
Gazebo / gazebo_ros2_control
    └── /joint_states
          ├── robot_state_publisher ──> /tf
          └── move_group 当前状态监视器

RViz MotionPlanning 的目标位姿或目标关节值
    └── move_group
          ├── KDL：正/逆运动学
          ├── Planning Scene：机器人状态与碰撞检查
          ├── OMPL / RRTConnect：搜索无碰撞关节路径
          └── 时间参数化：生成满足速度、加速度限制的轨迹
                └── FollowJointTrajectory Action
                      └── joint_trajectory_controller
                            └── position command interface
                                  └── GazeboSystem
                                        └── 机械臂运动
```

各组件职责：

- Gazebo 负责机械臂动力学和关节运动仿真。
- `joint_state_broadcaster` 发布六个关节的实际状态。
- `robot_state_publisher` 根据关节状态发布机器人 TF。
- `move_group` 负责运动学、碰撞检测、路径规划、时间参数化和轨迹执行管理。
- RViz MotionPlanning 是目标设置、规划预览和执行操作界面，本身不直接控制关节。
- `joint_trajectory_controller` 接收 MoveIt 生成的带时间戳六轴轨迹，并向 Gazebo position interface 输出命令。

#### 使用 RViz 进行规划

1. 启动联合仿真，等待终端出现：

   ```text
   You can start planning now!
   ```

2. 确认 Gazebo 已显示机械臂，RViz MotionPlanning 的 Planning Group 为 `cyx_arm`。
3. 在 MotionPlanning 的 **Planning** 页签选择起点：
   - 使用 **Current State**：以 Gazebo 当前姿态为起点；
   - 使用 **home**：采用 SRDF 中定义的六关节零位姿态。
4. 设置目标：
   - 拖动 `tool0` 的交互标记，指定目标位置和姿态；或
   - 在 **Joints** 页签直接输入六个目标关节角。
5. 点击 **Plan**：只计算并显示轨迹，不向控制器发送命令。
6. 检查 RViz 中的规划预览无误后点击 **Execute**，或直接使用 **Plan & Execute**。
7. 执行时 MoveIt 通过以下 Action 发送轨迹：

   ```text
   /joint_trajectory_controller/follow_joint_trajectory
   ```

8. Gazebo 执行轨迹后通过 `/joint_states` 返回实际位置，MoveIt 和 RViz 随之更新当前状态。

`tool0` 是相对 `arm_link6` 沿 +Z 偏移 16 mm 的空 TCP 坐标系。拖动交互标记时规划的是 `tool0` 位姿，不是 STL 几何中心或 `arm_joint6` 本身。

#### Plan 与 Execute 的区别

| 操作 | 结果 |
| --- | --- |
| **Plan** | 计算、碰撞检查并预览轨迹，不移动 Gazebo 机械臂 |
| **Execute** | 执行最近一次成功规划的轨迹 |
| **Plan & Execute** | 重新规划，成功后立即发送给控制器执行 |

调试新目标时建议先使用 **Plan** 检查轨迹，再使用 **Execute**。

#### 运行状态检查

检查控制器：

```bash
ros2 control list_controllers
```

预期输出包含：

```text
joint_state_broadcaster       joint_state_broadcaster/JointStateBroadcaster       active
joint_trajectory_controller  joint_trajectory_controller/JointTrajectoryController active
```

检查 MoveIt Action：

```bash
ros2 action list | grep -E 'move_action|execute_trajectory|follow_joint_trajectory'
```

检查当前关节状态：

```bash
ros2 topic echo /joint_states --once
```

如果 **Plan** 成功但 **Execute** 失败，应优先检查：

- `joint_trajectory_controller` 是否为 `active`；
- `/joint_states` 是否持续更新；
- `/joint_trajectory_controller/follow_joint_trajectory` 是否存在；
- 起点是否与 Gazebo 当前状态一致；
- 轨迹是否超过控制器容差或执行超时。

如果规划失败，应检查目标是否超出工作空间、是否超过关节限制，以及起点或目标是否发生自碰撞。

### 程序化运动控制

`cyx_motion_server` 在 MoveIt 之上提供稳定的项目接口。它维护统一运动状态机，并确保同一时刻只有一个规划、执行或键盘微动任务控制机械臂：

```text
IDLE → PLANNING → PLANNED → EXECUTING → SUCCEEDED
                    └───────────────→ FAILED / CANCELED
```

启动 MoveIt 联合仿真时，程序化规划服务器默认一起启动：

```bash
ros2 launch cyx_robot_moveit demo.launch.py
```

也可以控制是否启动：

```bash
ros2 launch cyx_robot_bringup bringup.launch.py \
  use_moveit:=true \
  use_motion_server:=true \
  use_keyboard_jog:=false
```

#### Action 接口

| Action | 用途 |
| --- | --- |
| `/cyx_arm/move_to_named` | 移动到命名姿态；当前只支持 `home` |
| `/cyx_arm/move_to_joints` | 六关节目标规划或执行 |
| `/cyx_arm/move_to_pose` | 指定末端 link 的目标位姿规划或执行 |
| `/cyx_arm/follow_cartesian` | 末端多路点笛卡尔路径 |
| `/cyx_arm/execute` | 按 `plan_id` 执行缓存轨迹 |

回到全关节零位：

```bash
ros2 action send_goal /cyx_arm/move_to_named \
  cyx_robot_interfaces/action/MoveToNamed \
  "{name: home, options: {velocity_scaling: 0.2, acceleration_scaling: 0.2}}" \
  --feedback
```

只规划六关节目标，不执行：

```bash
ros2 action send_goal /cyx_arm/move_to_joints \
  cyx_robot_interfaces/action/MoveToJoints \
  "{positions: [0.3, -0.2, 0.2, 0.0, 0.1, 0.0], options: {plan_only: true}}" \
  --feedback
```

规划并执行 `tool0` 目标位姿：

```bash
ros2 action send_goal /cyx_arm/move_to_pose \
  cyx_robot_interfaces/action/MoveToPose \
  "{target: {header: {frame_id: base_link}, pose: {position: {x: 0.15, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}, options: {end_effector_link: tool0, velocity_scaling: 0.2, acceleration_scaling: 0.2}}" \
  --feedback
```

规划末端笛卡尔多路点（示例仅规划，不执行）：

```bash
ros2 action send_goal /cyx_arm/follow_cartesian \
  cyx_robot_interfaces/action/FollowCartesianPath \
  "{waypoints: [{header: {frame_id: base_link}, pose: {position: {x: 0.15, y: 0.00, z: 0.35}, orientation: {w: 1.0}}}, {header: {frame_id: base_link}, pose: {position: {x: 0.15, y: 0.02, z: 0.35}, orientation: {w: 1.0}}}], options: {end_effector_link: tool0, plan_only: true}, eef_step: 0.005, jump_threshold: 0.0, minimum_fraction: 0.98}" \
  --feedback
```

只有路径完成比例达到 `minimum_fraction` 时轨迹才会被接受；用于实际执行前，应先结合机械臂当前姿态确认示例位点处于可达空间。

`end_effector_link` 没有硬编码为 `tool0`。请求未指定时采用参数 `default_end_effector_link`；未来加入夹爪、吸盘或相机后，可以传入 `gripper_tcp`、`suction_tcp` 等 URDF link，而无需修改 Action 类型。服务端会拒绝机器人模型中不存在的 link。

#### Service 接口

| Service | 用途 |
| --- | --- |
| `/cyx_arm/plan` | 通用 named/joint/pose 纯规划，返回轨迹和 `plan_id` |
| `/cyx_arm/stop` | 停止当前规划、轨迹执行或键盘微动 |
| `/cyx_arm/get_state` | 查询状态机、关节、末端位姿和最近错误 |

通用关节目标纯规划：

```bash
ros2 service call /cyx_arm/plan cyx_robot_interfaces/srv/Plan \
  "{goal_type: 1, joint_positions: [0.3, -0.2, 0.2, 0.0, 0.1, 0.0]}"
```

返回的 `plan_id` 可交给 `/cyx_arm/execute`：

```bash
ros2 action send_goal /cyx_arm/execute \
  cyx_robot_interfaces/action/ExecutePlan \
  "{plan_id: plan_1_123456789}" --feedback
```

缓存轨迹默认保存 300 秒、最多 10 条。执行前会检查机械臂当前状态是否仍接近原规划起点，避免在姿态已改变后执行旧轨迹。

查询与停止：

```bash
ros2 service call /cyx_arm/get_state cyx_robot_interfaces/srv/GetMotionState "{}"
ros2 service call /cyx_arm/stop cyx_robot_interfaces/srv/StopMotion "{}"
```

#### 公共规划选项

所有规划 Action 使用 `MotionOptions`：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `planning_group` | `cyx_arm` | MoveIt 规划组 |
| `end_effector_link` | `tool0` | 被控制的 TCP link，可替换 |
| `reference_frame` | `base_link` | 目标参考坐标系 |
| `planner_id` | `RRTConnectkConfigDefault` | OMPL 规划器 |
| `planning_time` | 5.0 s | 最大规划时间 |
| `planning_attempts` | 5 | 规划尝试次数 |
| `velocity_scaling` | 0.2 | 速度缩放，范围 `(0, 1]` |
| `acceleration_scaling` | 0.2 | 加速度缩放，范围 `(0, 1]` |
| `plan_only` | `false` | `true` 时只规划并缓存 |

### 键盘笛卡尔与关节微动

键盘节点应在独立终端中运行，以便读取按键：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cyx_robot_planning keyboard_jog.launch.py
```

该节点会直接读取当前控制终端 `/dev/tty`，因此可通过 `ros2 launch` 交互使用。按 `T` 在笛卡尔和关节模式之间切换，启动时默认为笛卡尔模式。有效按键被读取后终端会打印发布记录；如果出现 `No subscriber`，应先确认 `demo.launch.py` 中的 `cyx_motion_server` 已正常启动。若只看到输入字符回显而没有发布日志，请确认运行节点的是交互式终端，而不是无 TTY 的后台任务或重定向脚本。

默认每次按键让 `tool0` 相对 Gazebo/MoveIt 的模型坐标系 `world` 微动约 10 mm。当前 `base_link` 固定在 `world` 原点且方向一致，因此操作方向与机械臂基座坐标轴一致：

| 按键 | 方向 |
| --- | --- |
| `W` / `S` | X 正/负方向 |
| `A` / `D` | Y 正/负方向 |
| `I` / `K` | Z 正/负方向 |
| `T` | 切换到关节模式 |
| `Space` | 停止当前微动 |
| `Q` | 退出键盘节点 |

关节模式从 `arm_joint1` 开始选择：

| 按键 | 功能 |
| --- | --- |
| `A` / `D` | 循环选择上一个/下一个关节 |
| `W` / `S` | 当前关节角度增大/减小 |
| `T` | 切换回笛卡尔模式 |
| `Space` | 停止当前运动 |
| `Q` | 退出键盘节点 |

关节命令发布到 `/cyx_arm/joint_jog`。运动服务器读取当前六关节角，在选中关节上叠加 `joint_step`，然后使用 MoveIt 进行限位、自碰撞检查、规划和轨迹执行，不会绕过安全检查直接操作控制器。

当前关节步长为 10°；ROS 和 MoveIt 内部统一使用弧度，因此配置值为：

```yaml
# cyx_robot_planning/config/keyboard_jog.yaml
joint_step: 0.1745329252
```

一次按键的运行逻辑：

```text
读取当前末端位姿
  → 叠加 0.010 m 位移
  → 只设置 tool0 的 XYZ 位置目标，不设置姿态目标
  → MoveIt 自由选择末端姿态并进行关节空间规划
  → 检查 IK、碰撞和关节限制
  → joint_trajectory_controller 执行
```

键盘笛卡尔模式只约束目标位置，不保持当前末端姿态，因此比完整位姿约束更容易在奇异姿态附近找到解。相应地，该模式不保证末端姿态不变、不保证 TCP 沿严格直线移动，也可能选到与当前姿态差异较大的 IK 解；需要保持姿态或执行直线轨迹时，应使用 `/cyx_arm/move_to_pose` 或 `/cyx_arm/follow_cartesian`。

两种模式均采用“一次按键、一次安全规划和执行”，不允许按键命令堆积。其他 Action 正在运行时微动会被拒绝；微动执行期间其他运动请求也会被拒绝。末端 link、参考坐标系、笛卡尔步长和关节角步长可在 `cyx_robot_planning/config/keyboard_jog.yaml` 中修改。

即使只约束位置，目标仍可能因工作空间边界、自碰撞或关节限制而规划失败，服务器会安全拒绝该次微动。

### 自碰撞分析

使用固定随机种子在完整关节范围内统计自碰撞：

```bash
ros2 launch cyx_robot_moveit collision_analysis.launch.py \
  sample_count:=10000 random_seed:=42 use_acm:=true \
  output_file:=/tmp/cyx_collision_report.csv
```

`use_acm:=true` 用于验证实际 SRDF；设为 `false` 会忽略所有允许碰撞项，可用于重新生成候选矩阵。工具输出总碰撞比例、接近关节限位的样本比例和各 link 对频率。当前三个 `reason="Never"` 的 link 对来自 seed 42 的 20 万样本，不能替代机械结构变更后的重新采样和人工检查。

参数 `calculate_distance` 默认开启；若当前碰撞检测后端不提供有限距离，汇总中的 `minimum_distance` 为 `-1`。自动回归可启用 `strict:=true` 并设置 `max_collision_ratio`，超过阈值时进程返回失败。

### 可达空间可视化

一键启动 Gazebo、MoveIt、专用 RViz 和采样节点：

```bash
ros2 launch cyx_robot_bringup workspace_analysis.launch.py \
  sample_count:=50000 random_seed:=42 end_effector_link:=tool0
```

Marker 发布在 `/cyx_arm/reachable_workspace`，采用可靠、`TRANSIENT_LOCAL` QoS。默认过滤自碰撞姿态；黄色表示任一关节接近上下限，其他点由红到绿表示从低到高的雅可比可操作度。若设置 `include_colliding_samples:=true`，自碰撞样本显示为红色。

导出结果：

```bash
ros2 launch cyx_robot_bringup workspace_analysis.launch.py \
  use_rviz:=false use_gazebo_gui:=false sample_count:=50000 \
  output_csv:=/tmp/cyx_workspace.csv
```

CSV 包含六个关节角、TCP 的 X/Y/Z、自碰撞标志、近限位标志和可操作度。`end_effector_link` 可换成未来夹爪或吸盘的 TCP link，无需修改源码。该图表示“随机关节姿态对应的可达位置分布”，不代表每个位置能以任意姿态到达，也不等同于均匀笛卡尔体积采样。

发送一条五秒轨迹进行测试：

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [arm_joint1, arm_joint2, arm_joint3, arm_joint4, arm_joint5, arm_joint6], points: [{positions: [0.5, -0.4, 0.6, 0.3, -0.3, 0.4], time_from_start: {sec: 5}}]}"
```

## 推荐实现顺序

1. **模型描述（已完成）**：检查导出参数，将固定 URDF 拆分为 Xacro。
2. **模型显示（已完成）**：通过 `robot_state_publisher` 和 RViz 验证 TF 与关节运动。
3. **ros2_control（已完成）**：加入六关节 position command interface，以及 position、velocity state interface。
4. **控制器（已完成）**：配置 `joint_state_broadcaster` 和 `joint_trajectory_controller`。
5. **Gazebo 仿真（已完成）**：加载模型、控制器、GazeboSystem 和 RViz。
6. **MoveIt 2（已完成基础配置）**：配置 SRDF、KDL、OMPL、关节运行限制及控制器映射。
7. **程序化规划（已完成基础框架）**：提供 Action、Service、状态查询、轨迹缓存和键盘笛卡尔微动。
8. **统一启动（仿真已完成）**：由 `cyx_robot_bringup` 组合 Gazebo、模型、控制器、MoveIt、规划服务器和 RViz。
9. **碰撞与工作空间分析（已完成基础框架）**：简化碰撞体、采样验证 ACM，并提供碰撞统计和可达空间可视化。
10. **真实硬件**：实现 `DamiaoSystemHardware` 和 CAN 通信，使其可替换 Gazebo hardware 而不改变上层接口。

## 目标接口

当前提供：

- 轨迹输入：`/joint_trajectory_controller/joint_trajectory`
- 轨迹消息：`trajectory_msgs/msg/JointTrajectory`
- 关节状态：`/joint_states`
- 坐标变换：`/tf`
- 程序化规划：`/cyx_arm/move_to_named`、`/cyx_arm/move_to_joints`、`/cyx_arm/move_to_pose`、`/cyx_arm/follow_cartesian`
- 缓存轨迹执行：`/cyx_arm/execute`
- 状态与控制服务：`/cyx_arm/plan`、`/cyx_arm/stop`、`/cyx_arm/get_state`
- 笛卡尔微动输入：`/cyx_arm/cartesian_jog`
- 关节微动输入：`/cyx_arm/joint_jog`
- 可达空间 Marker：`/cyx_arm/reachable_workspace`

当前可用的启动命令：

```bash
ros2 launch cyx_robot_description display.launch.py
ros2 launch cyx_robot_simulation gazebo.launch.py
ros2 launch cyx_robot_bringup bringup.launch.py
ros2 launch cyx_robot_bringup simulation.launch.py
ros2 launch cyx_robot_bringup moveit_sim.launch.py
ros2 launch cyx_robot_bringup keyboard_control.launch.py
ros2 launch cyx_robot_bringup workspace_analysis.launch.py
ros2 launch cyx_robot_moveit demo.launch.py
ros2 launch cyx_robot_moveit collision_analysis.launch.py
```

## 后续达妙硬件规划（本阶段未实现）

`src/cyx_robot_hardware/config/damiao.yaml` 后续应为每个关节配置：

- 电机型号
- Motor ID
- CAN ID 与 CAN 接口
- 减速比
- 零位
- 方向
- 位置、速度和力矩限制

硬件插件计划继承 `hardware_interface::SystemInterface`，实现 ROS 2 Humble 对应生命周期以及 `read()`、`write()` 接口。具体函数签名应以服务器实际安装的 Humble API 为准。

## 开发约束

- 仿真控制与真实硬件控制保持解耦
- 不把上层控制逻辑直接写入 Gazebo 节点
- Gazebo 与达妙硬件使用一致的 joint/controller 接口
- 修改 mesh 文件或机械结构后，同步检查 URDF 惯量、碰撞体和坐标系
- 首次真机运行前必须设置合理的关节、速度、力矩和软件急停限制

## 说明

当前工作空间已在 Ubuntu 22.04 + ROS 2 Humble 环境完成 Xacro/URDF 解析、关节限制一致性检查、Python launch 语法检查和全包 `colcon build`。已验证 Gazebo 模型与控制器加载、MoveIt 关节规划及执行、缓存轨迹执行、运动状态查询，以及非奇异姿态下 `tool0` 的 10 mm 笛卡尔微动。
