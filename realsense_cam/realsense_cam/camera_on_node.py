import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import cv2
import pyrealsense2 as rs
from cv_bridge import CvBridge
import numpy as np

class cameraNode(Node):
    def __init__(self):
        super().__init__("cameraNode")

        self.rgb_publisher_ = self.create_publisher(Image, 'Image/realsense/RGB', 10)
        self.depth_publisher_ = self.create_publisher(Image, 'Image/realsense/Depth', 10)
        # self.pointcloud_publisher_ = self.create_publisher(PointCloud2, 'PointCloud/realsense', 10)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # 30 FPS in RGB format
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        self.pipeline.start(self.config)
        self.get_logger().info("Realsense camera Node on")

        self.timer_ = self.create_timer(0.5, self.publish_image)


    def publish_image(self):
        try:
            frame = self.pipeline.wait_for_frames()
            color_frame = frame.get_color_frame()
            depth_frame = frame.get_depth_frame()


            if not color_frame or not depth_frame:
                self.get_logger().warn("No color or depth frame received")
                return
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            ros_color_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            ros_depth_image = self.bridge.cv2_to_imgmsg(depth_image, "mono16")
            
            # pc2 = self.create_pointcloud(depth_image)

            # import pdb; pdb.set_trace()

            self.rgb_publisher_.publish(ros_color_image)
            self.depth_publisher_.publish(ros_depth_image)
            self.get_logger().info("Image published")

            # self.pointcloud_publisher_.publish(pc2)
        except Exception as e:
            self.get_logger().error(f"Error publishering image:{str(e)}")

    def create_pointcloud(self, depth_image):
        points = []

        # 获取深度相机的内参intrinsics
        depth_intrinsics = self.pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        # get_active_profile() 返回当前正在使用的配置文件，这个配置文件包含了当前激活的数据流配置（如颜色流、深度流等）
        # get_stream(rs.stream.depth) 从激活的配置文件中获取深度流配置
        # as_video_stream_profile() 将深度流配置转换为视频流配置
        # get_intrinsics() 从视频流配置中获取相机内参


        for y in range(depth_image.shape[0]):
            for x in range(depth_image.shape[1]):
                depth = depth_image[y, x]
                if depth == 0:
                    continue
                # Convert depth to real-world coordinates
                point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
                points.append([point[0], point[1], point[2]])

        # Convert points to PointCloud2 message
        header = self.create_header()
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        return point_cloud2.create_cloud(header, fields, points)

    def create_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "cam_link"
        return header


def main(args = None):
    rclpy.init(args = args)
    node = cameraNode()

    try:
        while rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()