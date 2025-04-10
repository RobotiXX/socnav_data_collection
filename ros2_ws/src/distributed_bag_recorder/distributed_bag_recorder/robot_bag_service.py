import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import subprocess
from datetime import datetime
import os 

class RobotBagService(Node):
    def __init__(self):
        super().__init__('robot_bag_service')
        self.create_service(Empty, 'start_recording', self.start_recording)
        self.create_service(Empty, 'stop_recording', self.stop_recording)
        self.recording_process = None

    def start_recording(self, request, response):
        timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        bag_path = f'/nvidia/home/ssd/bags/{timestamp}'
        os.makedirs(bag_path, exist_ok=True)
        topics = [
            '/joy',
            '/odom',
            '/cmd_vel',
            '/zed2/zed_node/rgb/camera_info',
            '/zed2/zed_node/rgb/image_rect_color/compressed',
            '/velodyne_points',
            '/witmotion_imu/imu',
            '/trajectory_marking',
            '/sync_command',
            '/tf',
            '/tf_static',
            '/zed2/zed_node/right/camera_info',
            '/zed2/zed_node/right/image_rect_color/compressed',
            '/insta360_x4/camera_info',
            '/insta360_x4/image_raw/compressed'
        ]

        self.recording_process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path, '--max-cache-size', '104857600'] + topics
        )
        self.get_logger().info(f"Started robot bag recording to {bag_path}")
        return response

    def stop_recording(self, request, response):
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.get_logger().info("Stopped robot bag recording.")
            self.recording_process = None
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotBagService()
    rclpy.spin(node)
    rclpy.shutdown()
