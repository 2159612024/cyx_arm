from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('cyx_robot_planning'), 'config', 'motion_server.yaml'
    ])
    return LaunchDescription([
        Node(
            package='cyx_robot_planning',
            executable='cyx_motion_server',
            name='cyx_motion_server',
            parameters=[config, {'use_sim_time': True}],
            output='screen',
        )
    ])
