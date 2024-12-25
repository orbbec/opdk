import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
from sensor_msgs_py import point_cloud2
import os
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('point_cloud_saver')

        # 定义点云订阅话题
        self.pointcloud_topics = [
            '/front_camera/depth/points',
            '/left_camera/depth/points',
            '/rear_camera/depth/points',
            '/right_camera/depth/points'
        ]

        self.subscribers = []
        self.file_counters = {topic: 1 for topic in self.pointcloud_topics}  # 为每个话题维护独立的计数器
        self.merged_counter = 1  # 融合点云的独立计数器

        # 存储点云数据用于融合
        self.cloud_data = {topic: None for topic in self.pointcloud_topics}

        # TF2 变换相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建订阅者
        for topic in self.pointcloud_topics:
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, topic=topic: self.callback(msg, topic),
                10
            )
            self.subscribers.append(sub)

        self.get_logger().info('PointCloudSaver initialized.')

    def callback(self, msg, topic):
        # 获取 frame_id
        frame_id = msg.header.frame_id
        self.get_logger().info(f"Received PointCloud2 from frame_id: {frame_id} on topic: {topic}")

        # 将 PointCloud2 转换为 numpy 数组
        cloud_array = self.pointcloud2_to_array(msg)

        # 获取外参变换并应用
        transform = self.get_transform(frame_id)
        if transform is not None:
            cloud_array = self.apply_transform(cloud_array, transform)

        # 保存到云数据中
        self.cloud_data[topic] = cloud_array

        # 生成递增文件名
        ply_filename = self.generate_incremental_filename(topic, frame_id)
        
        # 保存单独点云为 PLY 文件
        self.save_to_ply(cloud_array, ply_filename)
        self.get_logger().info(f"Saved {ply_filename}")

        # 检查是否所有点云都已接收
        if all(cloud is not None for cloud in self.cloud_data.values()):
            self.save_merged_ply()

    def get_transform(self, frame_id):
        reference_frames = ['base_link', 'odom', 'map']  # 尝试多个参考帧
        for ref_frame in reference_frames:
            try:
                transform_stamped = self.tf_buffer.lookup_transform(
                    ref_frame,  # 尝试的全局参考坐标系
                    frame_id,   # 点云的坐标系
                    rclpy.time.Time()
                )
                self.get_logger().info(f"Transform found for {frame_id} to {ref_frame}")
                return transform_stamped.transform
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"Could not get transform for {frame_id} to {ref_frame}: {e}")
        self.get_logger().error(f"Could not find transform for {frame_id} to any reference frame")
        return None

    def apply_transform(self, cloud_array, transform):
        # 将 Transform 转换为 4x4 变换矩阵
        t = transform.translation
        r = transform.rotation
        translation = np.array([t.x, t.y, t.z])
        rotation = np.array([r.x, r.y, r.z, r.w])  # 四元数

        # 计算旋转矩阵
        R = o3d.geometry.get_rotation_matrix_from_quaternion(rotation)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation

        # 应用变换
        cloud_array_h = np.hstack((cloud_array, np.ones((cloud_array.shape[0], 1))))  # 转为齐次坐标
        transformed_cloud = (T @ cloud_array_h.T).T[:, :3]  # 应用变换矩阵并返回点坐标
        return transformed_cloud

    def pointcloud2_to_array(self, cloud_msg):
        points = []
        for point in point_cloud2.read_points(cloud_msg, skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points, dtype=np.float32)

    def save_to_ply(self, cloud_array, filename):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(cloud_array)
        o3d.io.write_point_cloud(filename, pc)

    def save_merged_ply(self):
        # 融合所有点云
        merged_cloud = np.vstack([cloud for cloud in self.cloud_data.values() if cloud is not None])
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(merged_cloud)

        # 生成递增文件名
        merged_filename = f"merged_cloud_{self.merged_counter}.ply"
        self.merged_counter += 1

        # 保存融合点云
        o3d.io.write_point_cloud(merged_filename, pc)
        self.get_logger().info(f"Saved merged point cloud as {merged_filename}")

    def generate_incremental_filename(self, topic, frame_id):
        base_filename = f"{frame_id}"
        index = self.file_counters[topic]  # 使用独立计数器
        filename = f"{base_filename}_{index}.ply"

        # 更新计数器
        self.file_counters[topic] += 1
        return filename

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
