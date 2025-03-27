import argparse
import os
from pathlib import Path  # noqa: E402
import sys

# Hack to get relative import of .camera_config file working
from ament_index_python.packages import get_package_share_directory
USB_CAM_DIR = get_package_share_directory("usb_cam")
sys.path.append(os.path.join(USB_CAM_DIR, 'launch'))

from camera_config import CameraConfig  # noqa: E402
# from bev_cameras.utils.camera_config import CameraConfig, USB_CAM_DIR
from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='dell_wb3023',
        param_path=Path(get_package_share_directory('bev_cameras'), 'config', 'dell_wb3023.yaml')
    )
    # Add more Camera's here and they will automatically be launched below
)


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam dell_wb3023')
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
    # image_proc_nodes = [
    #     Node(
    #         package='image_transport',
    #         executable='republish',
    #         name=f'republish_to_raw_dell_wb3023',
    #         arguments=[
    #             'compressed', 'raw',
    #             '--ros-args',
    #             '-r', 'in/compressed:=/dell_wb3023/image_raw/compressed',
    #             '-r', 'out:=/dell_wb3023/republished_image_raw'
    #         ]
    #     ),
    #     Node(
    #         package='image_proc', executable='rectify_node',
    #         name='rectify_node',
    #         remappings=[
    #             ('image', '/dell_wb3023/republished_image_raw'),
    #             ('camera_info', '/dell_wb3023/camera_info'),
    #             ('image_rect', '/dell_wb3023/image_rect')
    #         ],
    #         parameters=[{
    #             "queue_size": 5,
    #             "image_transport": "compressed",
    #             "interpolation": 1}
    #         ]
    #     )
    # ]

    # camera_group = GroupAction(camera_nodes + image_proc_nodes)

    camera_group = GroupAction(camera_nodes)
    ld.add_action(camera_group)
    return ld