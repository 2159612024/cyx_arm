from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('cyx_arm', package_name='cyx_robot_moveit')
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution(file_path='config/controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )
    workspace_rviz = PathJoinSubstitution([
        FindPackageShare('cyx_robot_moveit'), 'config', 'workspace.rviz'
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('cyx_robot_moveit'), 'launch', 'demo.launch.py'
        ])),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': workspace_rviz,
            'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            'use_motion_server': 'false',
            'use_keyboard_jog': 'false',
            'world': LaunchConfiguration('world'),
        }.items(),
    )
    defaults = PathJoinSubstitution([
        FindPackageShare('cyx_robot_planning'), 'config', 'workspace_analysis.yaml'
    ])
    visualizer = Node(
        package='cyx_robot_planning',
        executable='cyx_workspace_visualizer',
        name='cyx_workspace_visualizer',
        output='screen',
        parameters=[
            defaults,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                'sample_count': LaunchConfiguration('sample_count'),
                'random_seed': LaunchConfiguration('random_seed'),
                'end_effector_link': LaunchConfiguration('end_effector_link'),
                'include_colliding_samples': LaunchConfiguration('include_colliding_samples'),
                'output_csv': LaunchConfiguration('output_csv'),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='false'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
            FindPackageShare('cyx_robot_simulation'), 'worlds', 'empty.world'
        ])),
        DeclareLaunchArgument('sample_count', default_value='50000'),
        DeclareLaunchArgument('random_seed', default_value='42'),
        DeclareLaunchArgument('end_effector_link', default_value='tool0'),
        DeclareLaunchArgument('include_colliding_samples', default_value='false'),
        DeclareLaunchArgument('output_csv', default_value=''),
        simulation,
        visualizer,
    ])
