from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_file = PathJoinSubstitution(
        [
            FindPackageShare('mecanum_bringup'),
            'config',
            'controllers.yaml',
        ]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', config_file],
    )

    jacobian = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['jacobian', '--param-file', config_file],
    )

    inverse_jacobian = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['inverse_jacobian', '--param-file', config_file],
    )

    return LaunchDescription(
        [
            controller_manager,
            joint_state_broadcaster,
            jacobian,
            inverse_jacobian,
        ]
    )
