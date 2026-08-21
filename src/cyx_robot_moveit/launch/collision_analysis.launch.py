from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    defaults = PathJoinSubstitution([
        FindPackageShare('cyx_robot_planning'), 'config', 'collision_analysis.yaml'
    ])
    analyzer = Node(
        package='cyx_robot_planning',
        executable='cyx_self_collision_analyzer',
        name='cyx_self_collision_analyzer',
        output='screen',
        parameters=[
            defaults,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                'sample_count': LaunchConfiguration('sample_count'),
                'random_seed': LaunchConfiguration('random_seed'),
                'use_allowed_collision_matrix': LaunchConfiguration('use_acm'),
                'calculate_distance': LaunchConfiguration('calculate_distance'),
                'strict': LaunchConfiguration('strict'),
                'max_collision_ratio': LaunchConfiguration('max_collision_ratio'),
                'output_file': LaunchConfiguration('output_file'),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('sample_count', default_value='10000'),
        DeclareLaunchArgument('random_seed', default_value='42'),
        DeclareLaunchArgument('use_acm', default_value='true'),
        DeclareLaunchArgument('calculate_distance', default_value='true'),
        DeclareLaunchArgument('strict', default_value='false'),
        DeclareLaunchArgument('max_collision_ratio', default_value='1.0'),
        DeclareLaunchArgument('output_file', default_value=''),
        analyzer,
    ])
