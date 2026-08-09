from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution(
        [FindPackageShare('cyx_robot_description'), 'urdf', 'cyx_robot.urdf.xacro']
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare('cyx_robot_control'), 'config', 'controllers.yaml']
    )
    robot_description = ParameterValue(
        Command(['xacro ', model, ' use_gazebo:=false']), value_type=str
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers],
        output='screen',
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        control_node,
        state_publisher,
        joint_state_broadcaster,
        trajectory_controller,
    ])
