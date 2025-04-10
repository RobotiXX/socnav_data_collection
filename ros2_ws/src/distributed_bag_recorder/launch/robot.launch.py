from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to scout_base launch file
    scout_base_dir = get_package_share_directory('scout_base')
    sensors_launch_path = os.path.join(scout_base_dir, 'launch', 'sensors.launch.py')

    return LaunchDescription([
        # Include sensor stack from scout_base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path)
        ),
        # Robot bag service node
        Node(
            package='distributed_bag_recorder',
            executable='robot_bag_service',
            name='robot_bag_service',
            output='screen'
        )
    ])
