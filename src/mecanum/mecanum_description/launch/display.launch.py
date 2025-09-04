from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():

    urdf_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare('mecanum_description'),
                'launch',
                'urdf.launch.py'
            ]
        )
    )

    view_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare('mecanum_description'),
                'launch',
                'view.launch.py'
            ]
        )
    )

    return LaunchDescription(
        [
            urdf_launch,
            view_launch
        ]
    )
