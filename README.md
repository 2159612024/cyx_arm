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
- 补齐 6 个功能包的 `package.xml` 和 `CMakeLists.txt`
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

尚未完成：

- MoveIt 自碰撞矩阵的完整采样优化及真实场景障碍物配置
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
| 机械限制参数源 | `src/cyx_robot_description/urdf/joint_limits.xacro` |
| 空 TCP 坐标系 | `src/cyx_robot_description/urdf/tool0.xacro` |
| 关节额定/峰值参数 | `src/cyx_robot_control/config/joint_parameters.yaml` |
| sw2urdf 原始辅助文件 | `src/cyx_robot_description/sw2urdf_export/` |

`sw2urdf_export/` 中的 package、CMake 和 launch 文件由 ROS 1 风格的导出器生成，仅作为原始资料保留，不应直接用于 ROS 2 构建。

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
ros2 launch cyx_robot_simulation gazebo.launch.py gui:=false use_rviz:=false
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
7. **真实硬件**：实现 `DamiaoSystemHardware` 和 CAN 通信，使其可替换 Gazebo hardware 而不改变上层接口。
8. **统一启动（仿真已完成）**：当前由 `cyx_robot_bringup` 组合 Gazebo、模型、控制器和 RViz；规划和硬件待后续扩展。

## 目标接口

当前提供：

- 轨迹输入：`/joint_trajectory_controller/joint_trajectory`
- 轨迹消息：`trajectory_msgs/msg/JointTrajectory`
- 关节状态：`/joint_states`
- 坐标变换：`/tf`

当前可用的启动命令：

```bash
ros2 launch cyx_robot_description display.launch.py
ros2 launch cyx_robot_simulation gazebo.launch.py
ros2 launch cyx_robot_bringup bringup.launch.py
ros2 launch cyx_robot_moveit demo.launch.py
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

当前工作空间已在 Ubuntu 22.04 + ROS 2 Humble 环境完成 Xacro/URDF 解析、关节限制一致性检查、Python launch 语法检查和全包 `colcon build`，并验证 mock hardware 和两个控制器能够完成加载与激活。Gazebo 与 RViz 联合仿真及 STL 模型显示已由项目使用者检查确认。
