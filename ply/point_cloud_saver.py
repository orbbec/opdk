import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PointCloudFusion(Node):
    def __init__(self):
        super().__init__('point_cloud_fusion')

        # 定义点云订阅话题
        self.front_topic = '/front_camera/depth/points'
        self.right_topic = '/right_camera/depth/points'

        # 创建订阅器
        self.front_sub = Subscriber(self, PointCloud2, self.front_topic)
        self.right_sub = Subscriber(self, PointCloud2, self.right_topic)

        # 使用 ApproximateTimeSynchronizer 进行时间同步
        self.ats = ApproximateTimeSynchronizer([self.front_sub, self.right_sub], queue_size=10, slop=0.1)
        self.ats.registerCallback(self.synchronized_callback)

        # TF2 变换相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化融合点云文件计数器
        self.merged_counter = 1
        self.front_counter = 1
        self.right_counter = 1

        self.get_logger().info('PointCloudFusion initialized.')

    def synchronized_callback(self, front_msg, right_msg):
        self.get_logger().info("Synchronized point clouds received.")

        # 转换 front_camera 点云
        front_cloud = self.pointcloud2_to_array(front_msg)
        front_transform = self.get_transform(front_msg.header.frame_id)
        if front_transform is not None:
            front_cloud = self.apply_transform(front_cloud, front_transform)

        # 转换 right_camera 点云
        right_cloud = self.pointcloud2_to_array(right_msg)
        right_transform = self.get_transform(right_msg.header.frame_id)
        if right_transform is not None:
            right_cloud = self.apply_transform(right_cloud, right_transform)

        # 保存单独的点云文件
        self.save_ply_file(front_cloud, 'front_camera', self.front_counter)
        self.save_ply_file(right_cloud, 'right_camera', self.right_counter)

        # 融合点云并保存
        self.save_merged_ply(front_cloud, right_cloud)

    def get_transform(self, frame_id):
        # 从 frame_id 转换到 base_link
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                frame_id,     # 点云的坐标系
                'base_link',  # 全局参考坐标系
                rclpy.time.Time()
            )
            self.get_logger().info(f"Transform found for {frame_id} to base_link.")
            return transform_stamped.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform for {frame_id} to base_link: {e}")
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
        cloud_array_h = np.hstack((cloud_array[:, :3], np.ones((cloud_array.shape[0], 1))))  # 转为齐次坐标
        transformed_cloud = (T @ cloud_array_h.T).T[:, :3]  # 应用变换矩阵并返回点坐标
        cloud_array[:, :3] = transformed_cloud
        return cloud_array

    def pointcloud2_to_array(self, cloud_msg):
        # 检查点云中是否包含 RGB 字段
        field_names = [field.name for field in cloud_msg.fields]
        with_rgb = 'rgb' in field_names

        points = []
        for point in point_cloud2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb") if with_rgb else ("x", "y", "z")):
            if with_rgb:
                x, y, z, rgb = point
                r = (np.uint32(rgb) >> 16) & 0x0000ff
                g = (np.uint32(rgb) >> 8) & 0x0000ff
                b = np.uint32(rgb) & 0x0000ff
                points.append([x, y, z, r / 255.0, g / 255.0, b / 255.0])  # 归一化 RGB 到 [0, 1]
            else:
                points.append([point[0], point[1], point[2]])
        return np.array(points, dtype=np.float32)

    def save_ply_file(self, cloud, camera_name, counter):
        # 创建 Open3D 点云对象
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(cloud[:, :3])  # 点的 XYZ 坐标
        if cloud.shape[1] == 6:  # 如果包含 RGB 信息
            pc.colors = o3d.utility.Vector3dVector(cloud[:, 3:6])

        # 保存为 PLY 文件
        ply_filename = f"{camera_name}_cloud_{counter}.ply"
        o3d.io.write_point_cloud(ply_filename, pc)
        self.get_logger().info(f"Saved {camera_name} point cloud as {ply_filename}")

        # 更新计数器
        if camera_name == 'front_camera':
            self.front_counter += 1
        elif camera_name == 'right_camera':
            self.right_counter += 1

    def save_merged_ply(self, front_cloud, right_cloud):
        # 合并点云
        merged_cloud = np.vstack((front_cloud, right_cloud))

        # 创建 Open3D 点云对象
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(merged_cloud[:, :3])  # 点的 XYZ 坐标
        if merged_cloud.shape[1] == 6:  # 如果包含 RGB 信息
            pc.colors = o3d.utility.Vector3dVector(merged_cloud[:, 3:6])

        # 保存为 PLY 文件
        merged_filename = f"merged_cloud_{self.merged_counter}.ply"
        self.merged_counter += 1
        o3d.io.write_point_cloud(merged_filename, pc)
        self.get_logger().info(f"Saved merged point cloud as {merged_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
