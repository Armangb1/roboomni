from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'robot_description_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mecanum_description'),
            'urdf',
            'robot.urdf.xacro'
        ]),
        description='Path to the robot description file'
    ))

    ld.add_action(DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mecanum_description'),
            'rviz',
            'config.rviz'
        ]),
        description='Path to the RViz configuration file'
    ))

    ld.add_action(DeclareLaunchArgument(
        'use_js_gui',
        default_value='false',
        description='Use joint state publisher GUI'
    ))

    # Include the robot state publisher
    robot_description = Command(
        ['xacro ', LaunchConfiguration('robot_description_file')])

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description,
                value_type=str
            )
        }]
    )
    )

    # Include the RViz node
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    ))

    # Include the joint state publisher GUI if specified
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_js_gui'))
    ))
    # Include the joint state publisher if GUI is not used
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_js_gui'))
    ))

    return ld
