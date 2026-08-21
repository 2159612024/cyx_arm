from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('cyx_robot_bringup'), 'launch', 'moveit_sim.launch.py'
        ])),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            'use_motion_server': 'true',
            'world': LaunchConfiguration('world'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='false'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
            FindPackageShare('cyx_robot_simulation'), 'worlds', 'empty.world'
        ])),
        bringup,
    ])
