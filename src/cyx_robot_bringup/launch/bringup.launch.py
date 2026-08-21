from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_moveit = LaunchConfiguration('use_moveit')
    use_motion_server = LaunchConfiguration('use_motion_server')
    use_keyboard_jog = LaunchConfiguration('use_keyboard_jog')
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_simulation'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            'world': LaunchConfiguration('world'),
        }.items(),
        condition=UnlessCondition(use_moveit),
    )
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cyx_robot_moveit'), 'launch', 'demo.launch.py'
            ])
        ),
        launch_arguments={
            'use_motion_server': use_motion_server,
            'use_keyboard_jog': use_keyboard_jog,
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            'world': LaunchConfiguration('world'),
        }.items(),
        condition=IfCondition(use_moveit),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('cyx_robot_simulation'), 'worlds', 'empty.world'
            ]),
        ),
        DeclareLaunchArgument('use_moveit', default_value='false'),
        DeclareLaunchArgument('use_motion_server', default_value='true'),
        DeclareLaunchArgument('use_keyboard_jog', default_value='false'),
        simulation_launch,
        moveit_launch,
    ])
