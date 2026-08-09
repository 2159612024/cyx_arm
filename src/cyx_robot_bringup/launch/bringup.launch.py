from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_simulation'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        simulation_launch,
    ])
