import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import subprocess
from datetime import datetime
import os
class HostBagService(Node):
    def __init__(self):
        super().__init__('host_bag_service')
        self.create_service(Empty, 'start_recording', self.start_recording)
        self.create_service(Empty, 'stop_recording', self.stop_recording)

        self.robot_start_cli = self.create_client(Empty, '/robot/start_recording')
        self.robot_stop_cli = self.create_client(Empty, '/robot/stop_recording')

        self.recording_process = None

    def start_recording(self, request, response):
        timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        bag_path = f'/home/scout/bags/{timestamp}'
        topics = [
            '/logitech_c920/image_raw/compressed',
            '/logitech_c920/camera_info',
        ]
        # os.makedirs(bag_path, exist_ok=True)

        self.recording_process = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_path, '--max-cache-size', '104857600'] + topics)
        self.get_logger().info(f"Started host bag recording to {bag_path}")

        while not self.robot_start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for robot start_recording service...')
        self.robot_start_cli.call_async(Empty.Request())

        return response

    def stop_recording(self, request, response):
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.get_logger().info("Stopped host bag recording.")
            self.recording_process = None

        while not self.robot_stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for robot stop_recording service...')
        self.robot_stop_cli.call_async(Empty.Request())

        return response

def main(args=None):
    rclpy.init(args=args)
    node = HostBagService()
    rclpy.spin(node)
    rclpy.shutdown()
