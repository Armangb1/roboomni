from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import NotSubstitution

def generate_launch_description():
    # Declare launch arguments for conditional execution
    use_kinect_arg = DeclareLaunchArgument(
        'use_kinect',
        default_value='true',
        description='Enable Kinect sensor'
    )
    
    use_display_arg = DeclareLaunchArgument(
        'use_display',
        default_value='false',
        description='Enable display/visualization'
    )
    
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable LiDAR sensor'
    )
    
    use_micro_arg = DeclareLaunchArgument(
        'use_micro',
        default_value='true',
        description='Enable microcontroller'
    )
    
    use_control_arg = DeclareLaunchArgument(
        'use_control',
        default_value='true',
        description='Enable control system'
    )
    
    # Original configuration arguments
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='lidar_link',
        description='Frame ID for LiDAR'
    )
    
    micro_port_arg = DeclareLaunchArgument(
        'micro_port',
        default_value='/dev/ttyUSB1',
        description='Port for microcontroller'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Port for LiDAR'
    )
    
    kinect_depth_frame_id_arg = DeclareLaunchArgument(
        'depth_frame_id',
        default_value='kinect_depth_optical_link',
        description='Frame ID for Kinect depth camera'
    )
    
    kinect_rgb_frame_id_arg = DeclareLaunchArgument(
        'rgb_frame_id',
        default_value='kinect_rgb_optical_link',
        description='Frame ID for Kinect RGB camera'
    )

    # Launch configurations
    use_kinect = LaunchConfiguration('use_kinect')
    use_display = LaunchConfiguration('use_display')
    use_lidar = LaunchConfiguration('use_lidar')
    use_micro = LaunchConfiguration('use_micro')
    use_control = LaunchConfiguration('use_control')
    
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    micro_port = LaunchConfiguration('micro_port')
    lidar_port = LaunchConfiguration('lidar_port')
    kinect_depth_frame_id = LaunchConfiguration('depth_frame_id')
    kinect_rgb_frame_id = LaunchConfiguration('rgb_frame_id')

    # Conditional launch inclusions
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mecanum_bringup'),
                'launch',
                'lidar_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'frame_id': lidar_frame_id,
            'serial_port': lidar_port
        }.items(),
        condition=IfCondition(use_lidar)
    )

    kinect_launch = IncludeLaunchDescription(
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
        }.items(),
        condition=IfCondition(use_kinect)
    )

    micro_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mecanum_bringup'),
                'launch',
                'micro_controller_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': micro_port
        }.items(),
        condition=IfCondition(use_micro)
    )
    
    # Placeholder for display launch (you'll need to create this launch file)
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mecanum_description'),
                'launch',
                'urdf.launch.py'
            ])
        ),
    )
    view_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mecanum_description'),
                'launch',
                'view.launch.py'
            ])
        ),
        condition=IfCondition(use_display),
        launch_arguments={'use_js': NotSubstitution(use_control)}.items()
    )
    
    # Placeholder for control launch (you'll need to create this launch file)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mecanum_bringup'),
                'launch',
                'ros2_control_bringup.launch.py'
            ])
        ),
        condition=IfCondition(use_control)
    )

    return LaunchDescription([
        # Declare all launch arguments
        use_kinect_arg,
        use_display_arg,
        use_lidar_arg,
        use_micro_arg,
        use_control_arg,
        lidar_frame_id_arg,
        micro_port_arg,
        lidar_port_arg,
        kinect_depth_frame_id_arg,
        kinect_rgb_frame_id_arg,
        
        urdf_launch,  # Always include URDF launch

        # Conditional launch inclusions
        lidar_launch,
        kinect_launch,
        micro_controller_launch,
        view_launch,
        control_launch,
    ])