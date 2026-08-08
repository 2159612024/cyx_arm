
# 六轴机械臂 ROS2 控制框架开发需求说明

## 1. 项目概述

本项目目标是在 ROS2 环境下搭建一个模块化六轴机械臂控制框架。

机械臂结构：

- 六自由度串联机械臂
- 构型：
  - 标准 Piper 机械臂构型
  - 6 个旋转关节
- 电机配置：
  - Joint1~Joint3：
    - 达妙 DM4340
  - Joint4~Joint6：
    - 达妙 DM4310

当前已有：

- 完成机械结构设计
- 已通过 SolidWorks + sw2urdf 导出 URDF 文件

后续目标：

1. 在 Gazebo 中完成机械臂动力学仿真
2. 在 RViz 中完成模型显示和运动控制
3. 建立标准 ROS2 控制接口
4. 支持 MoveIt2 运动规划
5. 保留硬件抽象层
6. 后续可以无缝替换为真实 DM4310/DM4340 电机控制
7. 支持扩展其他仿真平台


---

# 2. 开发环境要求


## ROS版本

目标：

```
ROS2 Humble
```

系统：

```
Ubuntu 22.04
```

主要依赖：

```
ros-humble-desktop

ros2_control

ros2_controllers

gazebo_ros2_control

moveit2

robot_state_publisher

joint_state_publisher

xacro
```


---

# 3. 工作空间结构


创建 ROS2 workspace：

```
<workspace_path>/
|
├── src/
│
│
├── cyx_robot_description/
│
├── cyx_robot_bringup/
│
├── cyx_robot_control/
│
├── cyx_robot_hardware/
│
├── cyx_robot_moveit/
│
└── cyx_robot_simulation/
```


说明：


## cyx_robot_description

负责：

- URDF/Xacro
- robot model
- mesh
- transmission
- ros2_control 描述


结构：

```
cyx_robot_description/

├── urdf/
│
│   ├── cyx_robot.urdf.xacro
│   ├── materials.xacro
│   ├── gazebo_control.xacro
│
├── meshes/
│
│   ├── link1.stl
│   ├── link2.stl
│   ...
│
├── launch/
│
│   └── display.launch.py
│
└── config/
```


URDF 文件位置：

```
<URDF_PATH>
```

由用户后续填写。


---

# 4. URDF处理要求


不要直接使用固定 URDF。


要求：

将原始 URDF 转换为 Xacro 管理。


需要：

```
cyx_robot.urdf.xacro
```


包含：

## robot_description


提供：

```
robot_state_publisher
```


## Gazebo插件


加入：

```
gazebo_ros2_control
```


## ros2_control


加入：

```
<ros2_control>
```


定义六个关节：

```
joint1

joint2

joint3

joint4

joint5

joint6
```


每个关节需要：

command interface:

```
position
```

state interface:

```
position

velocity
```


预留：

```
effort
```


---

# 5. ros2_control架构设计


采用标准结构：


```
controller_manager

        |
        |
        |

ros2_control

        |

hardware_interface

        |

DM4310/DM4340 driver

        |

CAN bus

```


当前阶段：

使用 Gazebo 仿真 hardware。


未来：

替换：

```
GazeboSystem
```

为：

```
DamiaoSystemHardware
```


---

# 6. 控制器设计


使用：

ros2_control。


配置：


## joint_state_controller


发布：

```
/joint_states
```


类型：

```
joint_state_broadcaster
```


---

## trajectory controller


使用：

```
joint_trajectory_controller
```


控制：

```
/joint_trajectory_controller
```


支持：

```
trajectory_msgs/JointTrajectory
```


---

配置文件：

路径：

```
cyx_robot_control/config/controllers.yaml
```


内容包含：

六轴控制器：

```yaml
joint_names:

 - joint1
 - joint2
 - joint3
 - joint4
 - joint5
 - joint6
```


---

# 7. Gazebo仿真


目标：

启动后：

```
Gazebo

+

RViz

+

robot_state_publisher

+

ros2_control
```


启动命令：


```
ros2 launch cyx_robot_simulation gazebo.launch.py
```


效果：

- Gazebo显示机械臂
- RViz显示TF
- 可以通过ROS2 topic控制关节


---

# 8. RViz配置


创建：

```
rviz/
|
└── cyx.rviz
```


显示：

必须包含：

```
RobotModel

TF

JointState

MotionPlanning
```


---

# 9. MoveIt2集成


创建：

```
cyx_robot_moveit
```


要求：

生成：

```
moveit_config
```


包含：


```
config/

├── controllers.yaml

├── joint_limits.yaml

├── kinematics.yaml

├── ompl_planning.yaml

└── cyx.srdf
```


支持：

- RViz MotionPlanning插件
- 关节空间规划
- 笛卡尔路径规划


---

# 10. 硬件抽象层设计


必须提前设计真实电机接口。


创建：

```
cyx_robot_hardware
```


结构：

```
cyx_robot_hardware/

├── include/

│
├── src/

│   └── damiao_hardware.cpp

│
├── config/

│
└── CMakeLists.txt
```



实现：

ROS2 hardware_interface:


继承：

```cpp
hardware_interface::SystemInterface
```



接口：

```cpp
configure()

activate()

deactivate()

read()

write()
```


---

# 11. 达妙电机接口规划


未来真实硬件：


电机：

```
Joint1 DM4340

Joint2 DM4340

Joint3 DM4340


Joint4 DM4310

Joint5 DM4310

Joint6 DM4310
```


接口：

```
CAN ID

Motor ID

Gear Ratio

Zero Position

Direction
```


配置文件：


```
damiao.yaml
```


格式：

```yaml
motors:

 joint1:

  type: DM4340

  id: 1

  can_id: 1


 joint2:

  type: DM4340

  id: 2


 joint3:

  type: DM4340

  id: 3


 joint4:

  type: DM4310

  id:4


 joint5:

  type: DM4310

  id:5


 joint6:

  type: DM4310

  id:6
```


---

# 12. Topic接口设计


## 输入控制


关节轨迹：

```
/joint_trajectory_controller/joint_trajectory
```


消息：

```
trajectory_msgs/JointTrajectory
```



## 状态输出


关节状态：

```
/joint_states
```


TF:

```
/tf
```


---

# 13. Launch设计


最终提供：


## 显示模型


```
ros2 launch cyx_robot_description display.launch.py
```


启动：

- robot_state_publisher
- RViz


---


## Gazebo仿真


```
ros2 launch cyx_robot_simulation gazebo.launch.py
```


启动：

- Gazebo
- ros2_control
- controllers


---


## MoveIt


```
ros2 launch cyx_robot_moveit demo.launch.py
```



---

# 14. Codex生成要求


请生成完整 ROS2 工程。


要求：


## 必须生成：

- package.xml
- CMakeLists.txt
- launch文件
- yaml配置
- xacro文件
- controller配置
- hardware_interface框架代码


---

## 编码规范


语言：

- Python:
  - launch文件


- C++:
  - hardware_interface


要求：

- ROS2 Humble规范
- 使用ament_cmake
- 使用现代C++17


---

# 15. 文件路径占位


以下路径由用户填写：


## ROS workspace

```
<WORKSPACE_PATH>
```


## 原始URDF

```
<URDF_PATH>
```


## Mesh文件

```
<MESH_PATH>
```


## 达妙SDK路径

```
<DAMIAO_SDK_PATH>
```


---

# 16. 最终目标


完成后系统应支持：

```
                 RViz

                  |

              MoveIt2

                  |

          ros2_control

                  |

        -----------------

        |               |

     Gazebo       DM Hardware

        |               |

   Simulation       Real Robot

```


最终实现：

同一套ROS2控制接口：

- Gazebo仿真
- RViz规划
- DM4310/DM4340真实机械臂控制

无需修改上层控制程序。



---

## 给 Codex 的执行要求

请按照以上设计生成完整 ROS2 Humble 工程。

优先完成：

1. robot_description
2. gazebo仿真
3. ros2_control
4. controller
5. MoveIt2接口
6. hardware_interface空实现

不要将控制逻辑直接写入Gazebo节点。

必须保持：

**仿真控制接口和真实硬件控制接口完全解耦。**

---

这份文档可以直接作为 Codex 的项目初始化说明。后续如果你把 **URDF 文件提供给 Codex 或上传给我**，还可以进一步生成：
- 精确的 xacro 拆分方案
- 六轴 joint 名称映射
- MoveIt SRDF
- ros2_control 参数
- DM4310/DM4340 CAN 驱动接口设计。