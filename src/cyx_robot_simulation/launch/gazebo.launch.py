from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world = LaunchConfiguration('world')
    model = PathJoinSubstitution(
        [FindPackageShare('cyx_robot_description'), 'urdf', 'cyx_robot.urdf.xacro']
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare('cyx_robot_control'), 'config', 'controllers.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('cyx_robot_description'), 'config', 'cyx.rviz']
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare('cyx_robot_simulation'), 'worlds', 'empty.world']
    )
    # Gazebo converts package:// mesh URIs to model:// URIs. Add the parent
    # share directory so model://cyx_robot_description resolves reliably.
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare('cyx_robot_description'), '..']),
            ':',
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ],
    )
    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' use_gazebo:=true',
            ' controllers_file:=', controllers,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'gui': use_gazebo_gui,
        }.items(),
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'cyx_arm', '-topic', 'robot_description'],
        output='screen',
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller', '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster, trajectory_controller],
        )
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        gazebo_model_path,
        gazebo,
        state_publisher,
        spawn_robot,
        start_controllers,
        rviz,
    ])
