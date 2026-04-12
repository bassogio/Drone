#!/usr/bin/env python3
import time
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data


class PointCloudNode(Node):

    def __init__(self):
        super().__init__('point_cloud_node')
        
        # ===============
        #  Parameters
        # ===============
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('point_cloud_topic', '/camera/depth/pointcloud')
        
        self.declare_parameter('output_frame', 'map')
        
        # Depth filtering and decimation
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 20.0)
        self.declare_parameter('pixel_step', 4)
        
        # Camera offset relative to drone body origin
        self.declare_parameter('camera_offset_x_m', 0.12)
        self.declare_parameter('camera_offset_y_m', 0.03)
        self.declare_parameter('camera_offset_z_m', 0.242)
        
        self.depth_image_topic = self.get_parameter('depth_image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.pixel_step = self.get_parameter('pixel_step').value
        self.min_depth_m = self.get_parameter('min_depth_m').value
        self.max_depth_m = self.get_parameter('max_depth_m').value
        
        self.camera_offset_body = np.array([
            float(self.get_parameter('camera_offset_x_m').value),
            float(self.get_parameter('camera_offset_y_m').value),
            float(self.get_parameter('camera_offset_z_m').value),
        ], dtype=np.float64)

        self.first_publish = True
        
        # ===============
        #  State
        # ===============
        # Current position and orientation of the drone
        self.current_position = None
        self.Qw = None
        self.Qx = None
        self.Qy = None
        self.Qz = None
        self.state_data_received = False
        
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
        
        self.position_ned = None   # [x, y, z] in NED
        self.q_body_to_ned = None  # [w, x, y, z]
  
        # ===============
        #  Subscribers
        # ===============
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10)
        
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            self.odometry_topic,
            self.odometry_callback,
            qos_profile_sensor_data)
        
        self.camera_depth_sub = self.create_subscription(
            Image,
            self.depth_image_topic,
            self.camera_depth_callback,
            10)
                        
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
        
    def odometry_callback(self, msg):
        self.current_position = np.array(msg.position,dtype=np.float64)
        self.Qw = msg.q[0]
        self.Qx = msg.q[1]
        self.Qy = msg.q[2]
        self.Qz = msg.q[3]
        self.state_data_received = True

    def camera_depth_callback(self, msg):
        if not self.camera_data_received or not self.state_data_received:
            missing = []
            if not self.camera_data_received:
                missing.append('camera')
            if not self.state_data_received:
                missing.append('odometry')

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
            strides= (msg.step, 4)
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

        uu = uu[valid].astype(np.float64)
        vv = vv[valid].astype(np.float64)
        z = z[valid].astype(np.float64)

        # ROS Camera optical frame (x right, y down, z forward)
        x_cam = (uu - self.cx) * z / self.fx
        y_cam = (vv - self.cy) * z / self.fy
        points_cam = np.stack((x_cam, y_cam, z), axis=1)
        
        # Camera optical -> PX4 body FRD
        # optical:  x right, y down, z forward
        # body FRD: x forward, y right, z down
        points_body = np.empty_like(points_cam)
        points_body[:, 0] = points_cam[:, 2]  # forward
        points_body[:, 1] = points_cam[:, 0]  # right
        points_body[:, 2] = points_cam[:, 1]  # down
        
        # Add static camera mount offset, expressed in body frame
        points_body += self.camera_offset_body
        
        # Body FRD to world NED using odometry data
        rotation_matrix = self.quaternion_to_rotation_matrix()
        points_ned = (rotation_matrix @ points_body.T).T + self.current_position

        # NED to ENU for RViz
        points_enu = np.empty_like(points_ned)
        points_enu[:, 0] = points_ned[:, 1]   # east
        points_enu[:, 1] = points_ned[:, 0]   # north
        points_enu[:, 2] = -points_ned[:, 2]  # up
        
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame
        
        # cloud_msg = point_cloud2.create_cloud_xyz32(
        #     header,
        #     points_enu.astype(np.float32).tolist()
        # )
        
        points_xyz = np.ascontiguousarray(points_enu, dtype=np.float32)

        cloud_msg = point_cloud2.create_cloud_xyz32(
            header,
            points_xyz
        )
        self.pointcloud_pub.publish(cloud_msg)
    
    def quaternion_to_rotation_matrix(self):
        w = self.Qw
        x = self.Qx
        y = self.Qy
        z = self.Qz

        # Normalize quaternion for safety
        n = math.sqrt(w*w + x*x + y*y + z*z)
        if n == 0.0:
            return np.eye(3, dtype=np.float64)

        w /= n
        x /= n
        y /= n
        z /= n

        r00 = 1.0 - 2.0 * (y*y + z*z)
        r01 = 2.0 * (x*y - z*w)
        r02 = 2.0 * (x*z + y*w)

        r10 = 2.0 * (x*y + z*w)
        r11 = 1.0 - 2.0 * (x*x + z*z)
        r12 = 2.0 * (y*z - x*w)

        r20 = 2.0 * (x*z - y*w)
        r21 = 2.0 * (y*z + x*w)
        r22 = 1.0 - 2.0 * (x*x + y*y)

        return np.array([
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22]
        ], dtype=np.float64)
    
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