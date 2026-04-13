#!/usr/bin/env python3
import time
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data


class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('map_node')
        
        # ===============
        #  Parameters
        # ===============
        self.declare_parameter('point_cloud_topic', '/camera/depth/point_cloud')
        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('occupancy_grid_topic', '/occupancy_map')

        self.declare_parameter('output_frame', 'map')
        
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.occupancy_grid_topic = self.get_parameter('occupancy_grid_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.first_publish = True
        
        # ===============
        #  State
        # ===============
        self.state_received = False
        # Current position and orientation of the drone
        self.current_position = None
        self.current_orientation = None
        
        # ===============
        #  Point Cloud
        # ===============
        self.point_cloud_received = False
        
        # ===============
        #  Map
        # ===============
        self.width = 400
        self.height = 400
        self.resolution = 0.1
        self.origin_x = 0.0
        self.origin_y = 0.0
        
        self.prior_probability = 0.5
        self.log_odds = np.full((self.height, self.width), self.prior_probability, np.float32)
        
        # ===============
        #  Subscribers
        # ===============
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            self.odometry_topic,
            self.odometry_callback,
            qos_profile_sensor_data)
                
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self.point_cloud_callback,
            qos_profile_sensor_data)
        
        # ===============
        #  Publishers
        # ===============
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.occupancy_grid_topic,
            10)
                
        self.get_logger().info('Occupancy grid node started')
        self.get_logger().info(f'Output frame: {self.output_frame}')
    
    def odometry_callback(self, msg):
        self.state_received = True
        self.current_position = np.array(msg.position,dtype=np.float32)
        self.current_orientation = np.array(msg.q,dtype=np.float32)
        
    def point_cloud_callback(self, msg):
        self.point_cloud_received = True
        
        if not self.state_received:
            self.get_logger().warn('Waiting for odometry data...')
            return
        
        points_cam = point_cloud2.read_points_numpy(msg, skip_nans=True).astype(np.float32)
        
        # In the following link: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html and also https://www.ros.org/reps/rep-0103.html
        # We can see that in the camera frame:
        # +x points to the right in the image
        # +y points down in the image
        # +z points into the plane of the image
        
        # In the following link: https://docs.px4.io/main/en/ros/external_position_estimation#reference-frames-and-ros
        # We can see that PX4 body frame is in FRD (X Forward, Y Right, Z Down)
        
        # Camera optical frame: x right, y down, z forward
        # body FRD: x forward, y right, z down
        # Converting points from camera optical frame to body FRD
        points_body = np.empty_like(points_cam)
        points_body[:, 0] = points_cam[:, 2]  # forward
        points_body[:, 1] = points_cam[:, 0]  # right
        points_body[:, 2] = points_cam[:, 1]  # down
        #! ADD CAMERA OFFSET
        
        # PX4 publishes odometry in North-East-Down (NED) navigation frame.
        # self.get_logger().info(f'pose_frame: {msg.pose_frame}') # Uncomment to see pose_frame values is 1
        # In order to convert from FRD to NED we apply a rotation matrix 
        rotation_matrix = self.quaternion_to_rotation_matrix()
        
        # points_body is stored as (N, 3):
        # [[x1, y1, z1],
        #  [x2, y2, z2],
        #  [x3, y3, z3]]
        # so each row is one point.
        # A rotation matrix is usually applied to column vectors, not row vectors.
        # So we transpose points_body to get shape (3, N):
        # [[x1, x2, x3],
        #  [y1, y2, y3],
        #  [z1, z2, z3]]
        # where each column is now one point.
        # Then we rotate all points at once using R @ P, and transpose back so the
        # result returns to the usual (N, 3) format, with one point per row.
        # Body FRD to world NED using odometry data
        points_ned = (self.rotation_matrix @ points_body.T).T + self.current_position
        
        # ROS world frame is in ENU (X East, Y North, Z Up)
        # NED: x north, y east, z down
        # ENU: x east, y north, z up
        # Converting points from NED to ENU
        points_enu = np.empty_like(points_ned)
        points_enu[:, 0] = points_ned[:, 1]  # east
        points_enu[:, 1] = points_ned[:, 0]  # north
        points_enu[:, 2] = -points_ned[:, 2] # up
        
        #! Add points filterring 
        
        
    def quaternion_to_rotation_matrix(self):
        w, x, y, z = self.current_orientation

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
        ], dtype=np.float32)
       
def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()