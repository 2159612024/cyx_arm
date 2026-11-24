# CYX Arm 项目移交说明（供本地 Codex 阅读）

> 最后更新：2026-08-26
> 项目目录：`~/cyx_workspace/cyx_arm`
> 目标平台：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic、MoveIt 2、ros2_control

## 1. 阅读顺序与协作约束

接手后请先阅读本文件和根目录 `README.md`，再修改代码。不要假定硬件参数已经确认。

用户此前明确的 Git 工作方式：

- 只更新本地 Git 仓库；
- 不要替用户推送 GitHub；
- 远端仓库为 `https://github.com/2159612024/cyx_arm.git`，分支为 `main`；
- 完成一个可独立验证的阶段后创建本地提交；
- 工作区可能包含用户自己的修改，修改前先执行 `git status --short`，不要覆盖无关变更。

开发环境约束：

- 不在 Conda `base` 环境中运行 ROS 2；
- `python3` 应使用 Ubuntu 系统 Python；
- 构建前执行 `source /opt/ros/humble/setup.bash`；
- 构建后执行 `source install/setup.bash`。

推荐初始化命令：

```bash
cd ~/cyx_workspace/cyx_arm
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
git status --short
```

## 2. 项目目标和当前边界

这是一个六自由度串联机械臂项目，关节名称固定为：

```text
arm_joint1
arm_joint2
arm_joint3
arm_joint4
arm_joint5
arm_joint6
```

计划使用的电机：

- Joint 1～3：达妙 DM4340；
- Joint 4～6：达妙 DM4310；
- 六个电机共用 CAN 总线；
- CAN 最终通过 USB2CAN 直接连接上位机；
- 没有下位机。

当前已经完成仿真、MoveIt 和上层控制框架，真实达妙硬件接口尚未实现。下一阶段重点是硬件层开发。

## 3. 软件包结构

```text
src/
├── cyx_robot_description   # Xacro/URDF、STL、关节限制、tool0、碰撞几何
├── cyx_robot_control       # ros2_control 控制器参数
├── cyx_robot_simulation    # Gazebo Classic 仿真
├── cyx_robot_moveit        # SRDF、KDL、OMPL、MoveIt/RViz 配置
├── cyx_robot_interfaces    # 项目 Action、Service、Message
├── cyx_robot_planning      # 运动服务器、键盘控制、碰撞/工作空间分析
├── cyx_robot_bringup       # 一键启动入口
└── cyx_robot_hardware      # 真实硬件占位包，下一阶段主要开发位置
```

## 4. 当前控制链路

```text
RViz / Action / Service / Keyboard
                ↓
         cyx_motion_server
                ↓
            move_group
                ↓
 joint_trajectory_controller
                ↓
       gazebo_ros2_control
                ↓
             Gazebo
```

接入实机后只应替换最下层：

```text
joint_trajectory_controller
                ↓
        ros2_control_node
                ↓
 cyx_robot_hardware/DamiaoSystemHardware
                ↓
       SocketCAN / USB2CAN
                ↓
          6 个达妙电机
```

上层 MoveIt、项目 Action/Service 和键盘节点应保持不变。

## 5. 模型与控制参数

- MoveIt planning group：`cyx_arm`；
- 规划链：`base_link -> tool0`；
- `tool0` 是无质量、无 visual、无 collision 的临时 TCP；
- `tool0` 相对 `arm_link6` 沿 Z 正方向偏移 `0.016 m`；
- `joint_limits.xacro` 是 URDF 与 ros2_control 的机械硬限制参数源；
- MoveIt 运行速度和估算加速度位于 `cyx_robot_moveit/config/joint_limits.yaml`；
- 当前 `joint_trajectory_controller` 使用 position command interface，以及 position/velocity state interface；
- `controller_manager.update_rate` 当前为 `100 Hz`。

运行时 collision 已由原始高面数 STL 改为 11 个基本几何体，定义在：

```text
src/cyx_robot_description/urdf/collision_geometry.xacro
```

这些碰撞体尺寸来自 STL 包围范围，是工程近似值，不是机械实测值。实机结构变化后必须重新检查。

## 6. 主要运行节点

- `robot_state_publisher`：发布 TF；
- `controller_manager`：管理 ros2_control 硬件和控制器；
- `joint_state_broadcaster`：发布 `/joint_states`；
- `joint_trajectory_controller`：执行六轴 FollowJointTrajectory；
- `move_group`：IK、碰撞检查、OMPL 规划和轨迹执行；
- `cyx_motion_server`：项目统一运动接口、状态机、轨迹缓存和互斥；
- `cyx_keyboard_jog`：读取 `/dev/tty`，提供笛卡尔/关节键盘微动；
- `cyx_self_collision_analyzer`：随机姿态碰撞统计；
- `cyx_workspace_visualizer`：可达空间 Marker 和 CSV。

## 7. 程序化接口

Action：

```text
/cyx_arm/move_to_named       # 当前仅支持 home（六关节零位）
/cyx_arm/move_to_joints
/cyx_arm/move_to_pose
/cyx_arm/follow_cartesian
/cyx_arm/execute             # 执行缓存 plan_id
```

Service：

```text
/cyx_arm/plan
/cyx_arm/stop
/cyx_arm/get_state
```

键盘话题：

```text
/cyx_arm/cartesian_jog
/cyx_arm/joint_jog
```

控制器接口：

```text
/joint_trajectory_controller/follow_joint_trajectory
/joint_trajectory_controller/joint_trajectory
/joint_states
```

## 8. 键盘控制的当前语义

启动命令：

```bash
ros2 launch cyx_robot_planning keyboard_jog.launch.py
```

节点显式打开 `/dev/tty`，因为 `ros2 launch` 不会把标准输入转发给子进程。

默认笛卡尔模式：

- `W/S`：X 正/负；
- `A/D`：Y 正/负；
- `I/K`：Z 正/负；
- 每次 `0.010 m`；
- `T` 切换模式；
- `Space` 停止；
- `Q` 退出。

笛卡尔键盘模式目前只设置 `tool0` 的 XYZ position target，不约束姿态。MoveIt 可以自由选择末端姿态，因此：

- 比完整 Pose 约束更容易求解；
- 不保证 TCP 走直线；
- 不保证末端姿态保持；
- 可能选择与当前关节状态差异较大的 IK 解；
- 零位处于最大伸展时，某些位置方向仍然不可达。

需要完整位姿约束时使用 `/cyx_arm/move_to_pose`；需要严格笛卡尔路点时使用 `/cyx_arm/follow_cartesian`。

关节模式：

- `A/D`：循环选择上一个/下一个关节；
- `W/S`：选中关节增加/减少 10°；
- 内部 SI 配置为 `joint_step: 0.1745329252 rad`；
- 指令仍经过 MoveIt 的限位、碰撞检查、规划和执行。

忙碌处理：

- 关节微动遇到运动服务器忙碌时不会直接丢弃；
- 只保留最新一条关节微动；
- 当前运动结束后自动执行；
- `Space` 和 `/cyx_arm/stop` 会清除待执行请求；
- 不允许形成无限命令队列。

## 9. 一键启动入口

```bash
# Gazebo + ros2_control + 可选 RViz
ros2 launch cyx_robot_bringup simulation.launch.py

# Gazebo + ros2_control + MoveIt + motion server + RViz
ros2 launch cyx_robot_bringup moveit_sim.launch.py

# 完整联合仿真 + 当前终端键盘控制
ros2 launch cyx_robot_bringup keyboard_control.launch.py

# 可达空间分析
ros2 launch cyx_robot_bringup workspace_analysis.launch.py

# 默认关闭 Gazebo GUI 的调试栈
ros2 launch cyx_robot_bringup debug.launch.py
```

常用无 GUI 参数：

```bash
ros2 launch cyx_robot_bringup moveit_sim.launch.py \
  use_rviz:=false use_gazebo_gui:=false
```

## 10. 已验证事项

- 8 个 ROS 2 软件包可完整 `colcon build`；
- Xacro 可展开，URDF 可通过 `check_urdf`；
- Gazebo 可加载机械臂及六轴 ros2_control；
- 两个控制器可以加载并激活；
- MoveIt 关节规划、执行和 plan cache 已验证；
- `tool0` 位置/姿态查询已验证；
- 键盘 `/dev/tty` 输入和模式切换已验证；
- 10°关节微动规划执行已验证；
- 连续两条关节微动可执行到约20°，忙碌时只保留最新请求；
- position-only 键盘目标沿可达方向执行成功；
- 自碰撞矩阵经过固定 seed 42、20 万随机样本辅助验证；
- 可达空间 Marker 和 CSV 导出已验证。

ROS 2 Humble 的 MoveIt/RViz 在 Ctrl-C 退出时偶尔出现 shutdown-only 的段错误或 class_loader 警告；正常运行与规划未因此失败。不要把退出阶段警告直接误判为运行时控制故障。

## 11. 当前已知限制

1. position-only 键盘微动可能选择较远 IK 解，不适合作为最终连续手动控制方案；长期建议评估 MoveIt Servo。
2. 键盘关节模式只缓存最新请求，笛卡尔微动忙碌时仍可能被拒绝。
3. `jog_eef_step`、`jog_minimum_fraction` 等旧参数在 position-only 键盘模式中已不再决定规划结果，可后续清理或重新命名。
4. 没有 3D 传感器，MoveIt 的 Octomap/`recognize_objects` 警告属于当前预期。
5. collision 基本几何体尚需结合最终机械结构人工复核。
6. `cyx_robot_hardware` 尚无真实驱动实现。
7. 达妙电机 CAN ID、主机 ID、固件协议版本、控制模式、减速比、零位、方向均未由实机确认。

## 12. 硬件层现状

`src/cyx_robot_hardware` 当前基本是占位包：

- `src/damiao_hardware.cpp` 尚未实现有效硬件插件；
- `config/damiao.yaml` 尚未包含实机确认参数；
- CMake 尚未构建 pluginlib 硬件插件；
- package.xml 尚未声明 hardware_interface/pluginlib 等依赖；
- Xacro 在非 Gazebo 模式下仍选择 `mock_components/GenericSystem`。

不要在缺少实机协议和安全参数时直接补写会驱动六轴运动的代码。

## 13. 推荐的硬件层架构

实现一个共享 CAN 总线的系统插件：

```cpp
class DamiaoSystemHardware
  : public hardware_interface::SystemInterface
```

建议拆分：

```text
cyx_robot_hardware/
├── include/cyx_robot_hardware/
│   ├── damiao_system_hardware.hpp
│   ├── damiao_protocol.hpp
│   ├── socketcan_transport.hpp
│   └── motor_state.hpp
├── src/
│   ├── damiao_system_hardware.cpp
│   ├── damiao_protocol.cpp
│   └── socketcan_transport.cpp
├── config/damiao.yaml
└── cyx_robot_hardware.xml
```

职责：

- `socketcan_transport`：非阻塞打开、过滤、发送、接收和错误帧处理；
- `damiao_protocol`：只负责厂商协议编解码，便于单元测试；
- `DamiaoSystemHardware`：ROS joint 与 motor 数值换算、生命周期、安全检查；
- 独立 CAN 接收线程：解析反馈并写入线程安全状态缓存；
- `read()`：非阻塞复制最新状态；
- `write()`：限幅并发送最新六轴命令。

生命周期建议：

- `on_init`：解析并严格校验六轴配置；
- `on_configure`：打开 CAN、设置过滤器、检查六电机在线；
- `on_activate`：先读取当前角度，再把 command 初始化为当前位置，最后使能；
- `read`：更新 position/velocity，建议同时预留 effort、温度和故障；
- `write`：检查 NaN、关节限制、单周期变化、通信状态并发送；
- `on_deactivate/on_error/on_shutdown`：停止、失能、关闭 CAN。

## 14. 硬件开发前必须确认

1. USB2CAN 是否在 Linux 下原生注册为 `can0`；若只出现 `/dev/ttyUSB*`，确认是 SLCAN 还是私有协议。
2. 实际 CAN 波特率。
3. 六个电机唯一 CAN ID 和反馈/主机 ID。
4. DM4340、DM4310 的固件版本和对应协议文档。
5. 第一版使用位置速度模式还是 MIT 模式；建议先使用电机内部位置闭环。
6. 电机反馈是转子侧还是输出轴侧数值。
7. 每轴减速比、方向和机械零位。
8. 电机侧通信超时看门狗行为。
9. 硬件急停方案。急停必须能独立切断驱动使能或动力，不能只依赖 CAN。

## 15. 推荐硬件开发顺序

1. 在 Linux 上确认 USB2CAN/SocketCAN 稳定工作。
2. 只连接一个卸载或可靠固定的电机。
3. 用厂商工具确认 ID、模式、限幅和反馈缩放。
4. 实现并单测达妙协议 pack/unpack，不接 ros2_control。
5. 实现 SocketCAN 非阻塞收发、过滤、错误帧和超时统计。
6. 编写单电机诊断程序，只做读取、使能、小角度运动、停止和失能。
7. 实现 `DamiaoSystemHardware`，先只导出 state interface。
8. 增加 position command interface，以低速度、低力矩、小角度测试。
9. 从一轴逐步扩展到六轴，测量 100 Hz 总线负载、反馈延迟和丢帧。
10. 增加 `hardware_mode:=mock|gazebo|real`，避免布尔参数误启动实机。
11. 新增 `hardware.launch.py` 和 `moveit_hardware.launch.py`，真实硬件入口不得启动 Gazebo。
12. 验证急停、USB断开、CAN bus-off、电机离线、节点崩溃和上位机重启。
13. 最后才允许 MoveIt 在低速度缩放下驱动整机。

## 16. 实机安全底线

- 第一次测试必须卸载、悬空或可靠固定机械结构；
- 使用低电流、低力矩、低速度和几度以内的目标；
- 操作人员必须能立即触及硬件急停；
- 任意电机反馈超时应停止全部六轴；
- USB 断开、CAN bus-off 或进程退出必须进入安全状态；
- 激活控制器时 command 必须初始化为实际位置，禁止从默认零位跳变；
- 未确认方向和零位前不得装载完整机械臂运行；
- 不得以软件 `/stop` 代替硬件急停。

## 17. 最近关键 Git 提交

```text
bc0221d Queue latest keyboard joint jog while busy
942cb20 Use position-only targets for Cartesian keyboard jog
729a73c Add keyboard joint jog mode
c98b148 Document collision and workspace analysis workflows
3f032f3 Add reachable workspace visualization
f93872a Add self-collision analysis tool
7195caf Generate and validate self-collision matrix
924d9e8 Optimize robot collision geometry
1d4701b Add unified simulation bringup launch files
4858a0f Fix keyboard jog input under ros2 launch
3db2ba9 Document and validate programmatic motion control
eaa3ebb Add programmatic MoveIt motion server and keyboard jog
```

## 18. 接手后的第一轮检查

本地 Codex 接手后建议依次执行：

```bash
git status --short
git log --oneline -12
which ros2
which python3
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch cyx_robot_bringup moveit_sim.launch.py
```

在开始真实硬件写操作前，先要求用户提供或确认：

- USB2CAN 型号和 Linux 识别结果；
- `ip -details link show` 或串口设备输出；
- 达妙官方协议文档版本；
- 六轴 CAN ID 表；
- 控制模式；
- 减速比、方向和零位；
- 电源、使能和硬件急停接线方案。

如果这些信息不完整，可以先实现纯协议单元测试和 `vcan` 虚拟总线测试，但不要发送真实电机运动命令。
