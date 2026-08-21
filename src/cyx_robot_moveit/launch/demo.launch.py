from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_motion_server = LaunchConfiguration('use_motion_server')
    use_keyboard_jog = LaunchConfiguration('use_keyboard_jog')
    moveit_config = (
        MoveItConfigsBuilder('cyx_arm', package_name='cyx_robot_moveit')
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution(file_path='config/controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .planning_scene_monitor(
            # The simulation robot_state_publisher owns /robot_description.
            # Publishing MoveIt's mock-hardware description here would race
            # spawn_entity and could omit gazebo_ros2_control from the model.
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_simulation'),
                'launch',
                'gazebo.launch.py',
            ])
        ),
        launch_arguments={'use_rviz': 'false'}.items(),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('cyx_robot_moveit'), 'config', 'moveit.rviz'
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    planning_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_planning'),
                'launch',
                'planning.launch.py',
            ])
        ),
        condition=IfCondition(use_motion_server),
    )
    keyboard_jog = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_planning'),
                'launch',
                'keyboard_jog.launch.py',
            ])
        ),
        condition=IfCondition(use_keyboard_jog),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_motion_server', default_value='true'),
        DeclareLaunchArgument('use_keyboard_jog', default_value='false'),
        simulation,
        move_group,
        rviz,
        planning_server,
        keyboard_jog,
    ])
