import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
from rclpy.qos import QoSPresetProfiles
import time


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")

        self.first_frame_timestamp = {}
        self.last_frame_timestamp = {}
        self.previous_timestamp = {}
        self.last_received_time = {}

        self.camera_list = [
            "front_camera",
            "left_camera",
            "rear_camera",
            "right_camera",
        ]
        self.ir_sides = ["left_ir", "right_ir"]
        self.shutdown = False
        self.default_diff = (
            0.023  # 设置上一帧与当前帧的时间戳间隔误差，超过35ms则打印输出
        )

        # self.bridge = CvBridge()

        for camera in self.camera_list:
            for ir_side in self.ir_sides:
                topic = f"/{camera}/{ir_side}/image_raw"
                self.create_subscription(
                    Image,
                    topic,
                    self.image_callback_factory(camera, ir_side),
                    QoSPresetProfiles.SENSOR_DATA.value,
                )
                self.get_logger().info(f"Subscribed to {topic}")
        self.create_timer(1.0, self.check_missing_data)

    def image_callback_factory(self, camera_name, ir_side):
        def callback(msg):
            topic_name = f"{camera_name}_{ir_side}"
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if topic_name not in self.first_frame_timestamp:
                self.first_frame_timestamp[topic_name] = current_time
                self.get_logger().info(
                    f"First frame timestamp for {topic_name}: {self.first_frame_timestamp[topic_name]}"
                )
            self.last_frame_timestamp[topic_name] = current_time
            if topic_name in self.previous_timestamp:
                time_diff = abs(current_time - self.previous_timestamp[topic_name])
                if time_diff > self.default_diff:
                    self.get_logger().info(
                        f"Time difference for {topic_name} between current and previous frame: {time_diff:.6f} seconds"
                    )
                    self.get_logger().info(
                        f"Current time: {current_time}, Previous time: {self.previous_timestamp[topic_name]}"
                    )
            self.previous_timestamp[topic_name] = current_time
            self.last_received_time[topic_name] = time.time()

        return callback

    def check_missing_data(self):
        current_time = time.time()
        for camera in self.camera_list:
            for ir_side in self.ir_sides:
                topic_name = f"{camera}_{ir_side}"
                if (
                    topic_name in self.last_received_time
                    and current_time - self.last_received_time[topic_name] > 2.0
                ):
                    if topic_name in self.last_frame_timestamp:
                        self.get_logger().warning(
                            f"No data received for {topic_name} in the last 2 seconds. "
                            f"Last frame timestamp: {self.last_frame_timestamp[topic_name]}"
                        )
                        self.shutdown = True
        if self.shutdown == True:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera subscriber...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
