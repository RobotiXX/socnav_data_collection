from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to bev_cameras launch file
    bev_cameras_dir = get_package_share_directory('bev_cameras')
    logitech_launch_path = os.path.join(bev_cameras_dir, 'launch', 'logitech_c920_launch.py')

    return LaunchDescription([
        # Include Logitech C920 camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(logitech_launch_path)
        ),
        # Host bag service node
        Node(
            package='distributed_bag_recorder',
            executable='host_bag_service',
            name='host_bag_service',
            output='screen'
        )
    ])
