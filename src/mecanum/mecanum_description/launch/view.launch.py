from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, NotSubstitution

def generate_launch_description():
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mecanum_description'),
            'rviz',
            'config.rviz'
        ]),
        description='Path to the RViz configuration file'
    )

    use_js_arg = DeclareLaunchArgument(
        'use_js',
        default_value='true',
        description='Use joint state publisher'
    )

    use_js_gui_arg = DeclareLaunchArgument(
        'use_js_gui',
        default_value='false',
        description='Use joint state publisher GUI'
    )

    rviz_config = LaunchConfiguration('rviz_config_file')

    # Include the RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Include the joint state publisher GUI if specified
    condition_js_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration('use_js'),
                LaunchConfiguration('use_js_gui')
            )
        ),
    )
    # Include the joint state publisher if GUI is not used
    condition_js = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(
                AndSubstitution(
                LaunchConfiguration('use_js'),
                NotSubstitution(LaunchConfiguration('use_js_gui'))
            )
        )
    )

    return LaunchDescription(
        [
            rviz_config_file_arg,
            use_js_arg,
            use_js_gui_arg,
            rviz,
            condition_js_gui,
            condition_js
        ]
    )