from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('cyx_robot_planning'), 'config', 'keyboard_jog.yaml'
    ])
    return LaunchDescription([
        Node(
            package='cyx_robot_planning',
            executable='cyx_keyboard_jog',
            name='cyx_keyboard_jog',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
