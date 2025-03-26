import argparse
import os
from pathlib import Path  # noqa: E402
import sys

# Hack to get relative import of .camera_config file working
from ament_index_python.packages import get_package_share_directory
USB_CAM_DIR = get_package_share_directory("usb_cam")
sys.path.append(os.path.join(USB_CAM_DIR, 'launch'))
from camera_config import CameraConfig  # noqa: E402

from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='insta360_x4',
        param_path=Path(get_package_share_directory('bev_cameras'), 'config', 'insta360_x4.yaml')
    )
    # Add more Camera's here and they will automatically be launched below
)


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='insta360_x4')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')

    camera_nodes = [
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings
        )
        for camera in CAMERAS
    ]

    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    return ld