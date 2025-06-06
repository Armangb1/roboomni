from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id', default='lidar_scan')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mecanum_bringup'),
                    'launch',
                    'lidar_bringup.launch.py'
                ])
            ),
            launch_arguments={
                'frame_id': frame_id
            }.items()
        ),

        Node(
            package="kinect_ros2",
            executable="kinect_ros2_node",
            name="kinect_ros2",
            namespace="kinect",
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttuUSB1"]
        )
    ])
