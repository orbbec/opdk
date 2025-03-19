import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import os
import shutil
from datetime import datetime

rosbag_duration = 30  # 30-second rolling buffer
odometry_timeout = 2.0  # Stop recording if odometry stops for 1 sec

topics_to_record = [
    "/front_camera/left_ir/camera_info",
    "/front_camera/left_ir/image_raw",
    "/front_camera/right_ir/camera_info",
    "/front_camera/right_ir/image_raw",
    "/left_camera/left_ir/camera_info",
    "/left_camera/left_ir/image_raw",
    "/left_camera/right_ir/camera_info",
    "/left_camera/right_ir/image_raw",
    "/rear_camera/left_ir/camera_info",
    "/rear_camera/left_ir/image_raw",
    "/rear_camera/right_ir/camera_info",
    "/rear_camera/right_ir/image_raw",
    "/right_camera/left_ir/camera_info",
    "/right_camera/left_ir/image_raw",
    "/right_camera/right_ir/camera_info",
    "/right_camera/right_ir/image_raw"
]

class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('rosbag_recorder')
        self.rosbag_process = None
        self.last_odometry_time = self.get_clock().now().nanoseconds / 1e9

        # Set rosbag save directory
        self.bag_directory = "/home/orbbec/Documents/orbbec/opdk/rosbag_temp"
        os.makedirs(self.bag_directory, exist_ok=True)

        # Start recording
        self.create_timer(rosbag_duration, self.restart_rosbag_recording)
        self.start_rosbag_recording()

        # Subscribe to odometry topic
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.odometry_callback, 10)

        # Timer to check odometry timeout
        self.create_timer(0.5, self.check_odometry_timeout)

    def start_rosbag_recording(self):
        if self.rosbag_process:
            self.stop_rosbag_recording()

        bag_name = os.path.join(self.bag_directory, "rolling_rosbag")

        # Ensure the rosbag file is removed before starting a new one
        if os.path.exists(bag_name):
            shutil.rmtree(bag_name, ignore_errors=True)

        cmd = ["ros2", "bag", "record", "-o", bag_name, "--max-bag-duration", str(rosbag_duration), "--storage", "sqlite3"]
        cmd.extend(topics_to_record)

        try:
            self.rosbag_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info(f"Started rosbag recording in {bag_name} with a rolling 30-second buffer.")
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag recording: {e}")

    def restart_rosbag_recording(self):
        self.get_logger().info("Restarting rosbag recording to maintain a 30-second buffer.")
        self.stop_rosbag_recording()
        self.start_rosbag_recording()

    def stop_rosbag_recording(self):
        if self.rosbag_process:
            self.rosbag_process.terminate()
            self.rosbag_process.wait()
            self.rosbag_process = None
            self.get_logger().info("Stopped rosbag recording.")

    def odometry_callback(self, msg):
        self.last_odometry_time = self.get_clock().now().nanoseconds / 1e9

    def check_odometry_timeout(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_odometry_time > odometry_timeout:
            self.get_logger().warn("Odometry topic stopped updating. Stopping recording.")
            self.stop_rosbag_recording()
            rclpy.shutdown()

    def destroy_node(self):
        self.stop_rosbag_recording()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

