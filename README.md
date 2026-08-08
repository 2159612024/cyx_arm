# cyx_arm

六自由度串联机械臂的 ROS 2 控制工程骨架。机械模型由 SolidWorks 设计，并通过 sw2urdf 导出。

本仓库当前用于在其他开发服务器上继续搭建 ROS 2 Humble、Gazebo、ros2_control、MoveIt 2 以及达妙电机硬件接口。仓库目前仅完成目录初始化和模型资产迁移，尚不具备编译、启动或控制能力。

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
- 创建 6 个功能包的目录骨架
- 迁移 sw2urdf 导出的 URDF、CSV、STL、配置、launch 和导出日志
- 将工作 URDF 的 mesh URI 从 `package://cyx_arm_001/...` 调整为 `package://cyx_robot_description/...`
- 保留 sw2urdf 原始导出辅助文件，方便后续追溯
- 初始化 Git 仓库并上传 GitHub

尚未完成：

- 各 ROS 2 包的 `package.xml` 和 `CMakeLists.txt`
- URDF 到 Xacro 的拆分与转换
- `ros2_control`、transmission 和 Gazebo 插件描述
- controller、RViz、Gazebo 和 MoveIt 2 配置
- Python launch 文件
- 达妙电机 `hardware_interface` 实现
- 编译、仿真和真机测试

除迁移的 sw2urdf 文件外，仓库中的代码及配置骨架文件有意保持为空。

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
| 待实现的 Xacro | `src/cyx_robot_description/urdf/cyx_robot.urdf.xacro` |
| sw2urdf 原始辅助文件 | `src/cyx_robot_description/sw2urdf_export/` |

`sw2urdf_export/` 中的 package、CMake 和 launch 文件由 ROS 1 风格的导出器生成，仅作为原始资料保留，不应直接用于 ROS 2 构建。

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
- MoveIt 2
- `colcon`

克隆仓库：

```bash
git clone https://github.com/2159612024/cyx_arm.git
cd cyx_arm
```

当前版本不能执行 `colcon build`。应先补全各包的 `package.xml`、`CMakeLists.txt` 和源文件，再使用：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 推荐实现顺序

1. **模型描述**：检查质量、惯量、关节轴、坐标系和碰撞模型，将固定 URDF 拆分为 Xacro。
2. **模型显示**：实现 `cyx_robot_description`，通过 `robot_state_publisher` 和 RViz 验证 TF 与关节运动。
3. **ros2_control**：加入六关节 position command interface，以及 position、velocity state interface；预留 effort。
4. **控制器**：配置 `joint_state_broadcaster` 和 `joint_trajectory_controller`。
5. **Gazebo 仿真**：实现 `cyx_robot_simulation`，加载模型、控制器和仿真硬件。
6. **MoveIt 2**：生成 SRDF、运动学、关节限制、规划器及控制器映射。
7. **真实硬件**：实现 `DamiaoSystemHardware` 和 CAN 通信，使其可替换 Gazebo hardware 而不改变上层接口。
8. **统一启动**：由 `cyx_robot_bringup` 组合模型、控制器、规划和硬件启动流程。

## 目标接口

计划提供：

- 轨迹输入：`/joint_trajectory_controller/joint_trajectory`
- 轨迹消息：`trajectory_msgs/msg/JointTrajectory`
- 关节状态：`/joint_states`
- 坐标变换：`/tf`

计划提供的启动命令：

```bash
ros2 launch cyx_robot_description display.launch.py
ros2 launch cyx_robot_simulation gazebo.launch.py
ros2 launch cyx_robot_moveit demo.launch.py
```

以上命令目前只是目标接口，相关文件仍为空，尚不可执行。

## 达妙硬件规划

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

该仓库尚未在 ROS 2 环境中构建或测试。后续提交应在 Ubuntu 22.04 + ROS 2 Humble 服务器上完成编译、launch、RViz、Gazebo、MoveIt 2 和硬件测试，并在 README 中持续更新实际可用命令与测试结果。
