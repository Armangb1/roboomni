from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_frame_id',
            default_value='kinect_rgb_optical_link',
            description='Frame ID for the RGB camera'
        ),
        DeclareLaunchArgument(
            'depth_frame_id',
            default_value='kinect_depth_optical_link',
            description='Frame ID for the Depth camera'
        ),
        Node(
            package='kinect_ros2',
            executable='kinect_ros2_node',
            name='kinect',
            output='screen',
            parameters=[{
                'rgb_frame_id': LaunchConfiguration('rgb_frame_id'),
                'depth_frame_id': LaunchConfiguration('depth_frame_id')
            }],
        )
    ])
