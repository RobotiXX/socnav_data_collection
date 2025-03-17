#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    usb_cam_params = {
        # Services used to start/stop the camera capture
        'start_service_name': 'start_capture',
        'stop_service_name': 'stop_capture',

        # Camera device and I/O settings
        'video_device': '/dev/video2',
        'io_method': 'mmap',
        'pixel_format': 'mjpeg2rgb',
        'color_format': 'yuv420p',
        'image_width': 2560,
        'image_height': 1440,
        'framerate': 30.0,

        # Behavior flags and logging
        'create_suspended': False,
        'full_ffmpeg_log': False,

        # Naming and transforms
        'camera_name': 'dell_camera',
        'camera_frame_id': 'dell_camera',
        'camera_transport_suffix': 'image_raw',
        'camera_info_url': '',

        # Intrinsic V4L controls
        'intrinsic_controls': {
            'autofocus': True,
            'exposure_auto_priority': True,
            'autoexposure': 3,
            'white_balance_temperature_auto': True,
            'power_line_frequency': 60,
            'ignore': [
                'brightness',
                'contrast',
                'saturation',
                'gain',
                'sharpness',
                'backlight_compensation',
                'white_balance_temperature',
                'exposure_absolute',
                'pan_absolute',
                'tilt_absolute',
                'focus_absolute',
                'zoom_absolute'
            ]
        }
    }

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',  # ROS node name
        output='screen',
        parameters=[usb_cam_params],
    )

    return LaunchDescription([usb_cam_node])
