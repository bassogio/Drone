#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data


class PointCloudNode(Node):

    def __init__(self):
        super().__init__('point_cloud_node')
        
        # ===============
        #  Parameters
        # ===============
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('point_cloud_topic', '/camera/depth/point_cloud')
        
        self.declare_parameter('output_frame', 'camera_optical_frame')
        
        # Depth filtering and decimation
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 20.0)
        self.declare_parameter('pixel_step', 4)
        
        self.depth_image_topic = self.get_parameter('depth_image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.pixel_step = self.get_parameter('pixel_step').value
        self.min_depth_m = self.get_parameter('min_depth_m').value
        self.max_depth_m = self.get_parameter('max_depth_m').value

        self.first_publish = True
        
        # ===============
        #  Camera
        # ===============
        # Camera intrinsic parameters
        self.k = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_data_received = False
  
        # ===============
        #  Subscribers
        # ===============
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data)
        
        self.camera_depth_sub = self.create_subscription(
            Image,
            self.depth_image_topic,
            self.camera_depth_callback,
            qos_profile_sensor_data)
                        
        # ===============
        #  Publishers
        # ===============
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.point_cloud_topic,
            10)
        
        self.get_logger().info('Depth to point cloud node started')
        self.get_logger().info(f'Output frame: {self.output_frame}')
        
    def camera_info_callback(self, msg):
        self.k = msg.k
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.camera_data_received = True

    def camera_depth_callback(self, msg):
        data = msg.data
        
        if not self.camera_data_received or data is None:
            missing = []
            if not self.camera_data_received:
                missing.append('camera')
            if data is None:
                missing.append('depth image')
            self.get_logger().warning(f"Waiting for {' and '.join(missing)} data")
            return
        
        if msg.encoding != '32FC1':
            self.get_logger().error(f'Unsupported depth encoding: {msg.encoding}')
            return
        
        if self.first_publish:
            self.get_logger().info('Publishing cloud.')
            self.first_publish = False
        
        data = msg.data
        
        # Convert the raw depth-image byte buffer into a 2D NumPy array.
        # shape   - image height and width
        # dtype   - each pixel is a float32 depth value
        # strides - describes how to move through memory:
        #           msg.step bytes to the next row
        #           4 bytes to the next column, because float32 = 4 bytes
        depth = np.ndarray(
            buffer=data,
            shape=(msg.height, msg.width),
            dtype=np.float32, 
            strides=(msg.step, 4)
        )
        
        # Sample pixel rows and columns with a step to reduce the number of depth points
        v_coords = np.arange(0, msg.height, self.pixel_step, dtype=np.int32)
        u_coords = np.arange(0, msg.width, self.pixel_step, dtype=np.int32)

        # Create a 2D grid of sampled pixel coordinates:
        # uu - horizontal pixel positions
        # vv - vertical pixel positions
        uu, vv = np.meshgrid(u_coords, v_coords)

        z = depth[vv, uu]
        
        valid = np.isfinite(z)
        valid = valid & (z >= self.min_depth_m)
        valid = valid & (z <= self.max_depth_m)
        
        if not np.any(valid):
            return

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        # ROS Camera optical frame (x right, y down, z forward)
        x_cam = (uu - self.cx) * z / self.fx
        y_cam = (vv - self.cy) * z / self.fy
        points_cam = np.stack((x_cam, y_cam, z), axis=1)
        
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame
        
        points_xyz = np.ascontiguousarray(points_cam, dtype=np.float32)

        cloud_msg = point_cloud2.create_cloud_xyz32(
            header,
            points_xyz
        )
        self.pointcloud_pub.publish(cloud_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()