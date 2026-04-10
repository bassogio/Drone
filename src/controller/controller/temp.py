#!/usr/bin/env python3

import time
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from rclpy.qos import qos_profile_sensor_data

class PointCloudNode(Node):

    def __init__(self):
        super().__init__('point_cloud_node')
        
        # ===============
        #  Parameters
        # ===============
        # Current position and orientation of the drone
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        self.Qx = None
        self.Qy = None
        self.Qz = None
        self.Qw = None       

        # Camera intrinsic parameters
        self.k = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.depth_image = None
        self.camera_height = None
        self.camera_width = None
        
        
        self.points = None
        
        self.timer_period = 0.1 # seconds
        
        # ===============
        #  Subscribers
        # ===============
        self.camera_depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.camera_depth_callback,
            10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback,
            qos_profile_sensor_data)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos_profile_sensor_data
        )
                
        # ===============
        #  Publishers
        # ===============
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/camera/depth/pointcloud',
            10)
    
    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading
        
    def attitude_callback(self,msg):
        self.Qw = msg.q[0]
        self.Qx = msg.q[1]
        self.Qy = msg.q[2]
        self.Qz = msg.q[3]
        
    def quaternion_to_rotation_matrix(self):
        q0 = self.Qw
        q1 = self.Qx
        q2 = self.Qy
        q3 = self.Qz
        
        r00 = 2 * (q0 ** 2 + q1 ** 2) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 ** 2 + q2 ** 2) - 1
        r12 = 2 * (q1 * q3 - q0 * q1)
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1) 
        r22 = 2 * (q0 ** 2 + q3 ** 2) - 1
        
        self.rot_matrix = np.array([[r00, r01, r02],
                                    [r10, r11, r12],
                                    [r20, r21, r22]])
        
    def camera_depth_callback(self, msg):
        self.camera_height = msg.height
        self.camera_width = msg.width

        raw_depth = np.frombuffer(msg.data, dtype=np.float32)
        self.depth_image = raw_depth.reshape((msg.height, msg.width))
        
        self.points = self.create_points_from_depth()
        
        if self.points is None:
            return
        
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header=msg.header,
            points=self.points
        )
        self.pointcloud_pub.publish(cloud_msg)

    def camera_info_callback(self, msg):
        self.k = msg.k
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        
    def create_points_from_depth(self):
        points = []
        
        for v in range(self.camera_height):
            for u in range(self.camera_width):
                depth = float(self.depth_image[v, u])

                # Skip invalid depth values
                if not np.isfinite(depth) or depth <= 0.0:
                    continue
                
                # Convert depth image pixel (u, v) to 3D point (x, y, z) in camera frame
                z = depth
                x = (float(u) - self.cx) * z / self.fx
                y = (float(v) - self.cy) * z / self.fy                
                
                points.append((x, y, z))
        
        return points
                
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