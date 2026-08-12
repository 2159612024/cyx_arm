from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_moveit = LaunchConfiguration('use_moveit')
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_simulation'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items(),
        condition=UnlessCondition(use_moveit),
    )
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_moveit'), 'launch', 'demo.launch.py'
            ])
        ),
        condition=IfCondition(use_moveit),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_moveit', default_value='false'),
        simulation_launch,
        moveit_launch,
    ])
