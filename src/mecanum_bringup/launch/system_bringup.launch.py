from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar_frame_id = LaunchConfiguration(
        'lidar_frame_id', default='lidar_link')
    micro_port = LaunchConfiguration('micro_port', default='/dev/ttyUSB0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB1')
    kinect_depth_frame_id = LaunchConfiguration(
        'depth_frame_id', default='kinect_depth_optical_link')
    kinect_rgb_frame_id = LaunchConfiguration(
        'rgb_frame_id', default='kinect_rgb_optical_link')

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
                'frame_id': lidar_frame_id,
                'lidar_port': lidar_port
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mecanum_bringup'),
                    'launch',
                    'kinect_bringup.launch.py'
                ])
            ),
            launch_arguments={
                'depth_frame_id': kinect_depth_frame_id,
                'rgb_frame_id': kinect_rgb_frame_id
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mecanum_bringup'),
                    'launch',
                    'micro_controller_bringup.launch.py'
                ])
            ),
            launch_arguments={
                'micro_port': micro_port
            }.items()
        )
    ])
