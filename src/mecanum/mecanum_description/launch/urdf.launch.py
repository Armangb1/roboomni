from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    
    urdf_file_arg = DeclareLaunchArgument(
        'robot_description_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mecanum_description'),
            'urdf',
            'robot.urdf.xacro'
        ]),
        description='Path to the robot description file'
    )

    urdf_path = LaunchConfiguration('robot_description_file')
    # Include the robot state publisher
    robot_description = Command(
        ['xacro ', urdf_path]
    )

    robot_state_publisher = Node(
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
    return LaunchDescription(
        [
            urdf_file_arg,
            robot_state_publisher
        ]
    )