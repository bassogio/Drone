#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header


class PointCloudNode(Node):

    def __init__(self):
        super().__init__('point_cloude_node')

        # ===============
        # Parameters
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

        # ===============
        # Subscribers
        # ===============
        self.camera_depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.camera_depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos_profile_sensor_data
        )

        # ===============
        # Publishers
        # ===============
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/camera/depth/pointcloud',
            10
        )

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    def attitude_callback(self, msg):
        self.Qw = msg.q[0]
        self.Qx = msg.q[1]
        self.Qy = msg.q[2]
        self.Qz = msg.q[3]

    def quaternion_to_rotation_matrix(self):
        if None in (self.Qw, self.Qx, self.Qy, self.Qz):
            return None

        q = np.array([self.Qw, self.Qx, self.Qy, self.Qz], dtype=np.float64)
        norm = np.linalg.norm(q)
        if norm == 0.0:
            return None

        w, x, y, z = q / norm

        # Rotation from PX4 body FRD -> PX4 world NED
        R = np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)]
        ], dtype=np.float64)

        return R

    def camera_optical_to_body_frd(self, p_cam):
        """
        Convert point from ROS optical frame to PX4 body FRD.

        ROS optical:
            x = right
            y = down
            z = forward

        PX4 body FRD:
            x = forward
            y = right
            z = down

        Therefore:
            body_x = cam_z
            body_y = cam_x
            body_z = cam_y
        """
        R_body_cam = np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ], dtype=np.float64)

        # Camera translation relative to body frame.
        # Start with zero. Later, replace with the actual offset if needed.
        t_body_cam = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        return R_body_cam @ p_cam + t_body_cam

    def ned_to_enu(self, p_ned):
        """
        Convert PX4 NED world coordinates to ROS ENU coordinates.
        NED: x=north, y=east, z=down
        ENU: x=east, y=north, z=up
        """
        return np.array([p_ned[1], p_ned[0], -p_ned[2]], dtype=np.float64)

    def camera_point_to_world(self, x_cam, y_cam, z_cam):
        if None in (self.current_x, self.current_y, self.current_z):
            return None

        R_world_body = self.quaternion_to_rotation_matrix()
        if R_world_body is None:
            return None

        p_cam = np.array([x_cam, y_cam, z_cam], dtype=np.float64)

        # camera optical -> body FRD
        p_body = self.camera_optical_to_body_frd(p_cam)

        # body FRD -> world NED
        t_world_body = np.array(
            [self.current_x, self.current_y, self.current_z],
            dtype=np.float64
        )

        p_world_ned = R_world_body @ p_body + t_world_body

        # world NED -> world ENU for ROS/RViz
        p_world_enu = self.ned_to_enu(p_world_ned)

        return p_world_enu

    def camera_depth_callback(self, msg):
        self.camera_height = msg.height
        self.camera_width = msg.width

        raw_depth = np.frombuffer(msg.data, dtype=np.float32)
        self.depth_image = raw_depth.reshape((msg.height, msg.width))

        self.points = self.create_points_from_depth()

        if self.points is None:
            return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'map'

        cloud_msg = point_cloud2.create_cloud_xyz32(
            header=header,
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

                # Convert pixel (u, v) to 3D point in camera optical frame
                z = float(depth)
                x = (float(u) - float(self.cx)) * z / float(self.fx)
                y = (float(v) - float(self.cy)) * z / float(self.fy)

                # Convert camera-frame point to world-frame point
                p_world = self.camera_point_to_world(x, y, z)
                if p_world is None:
                    continue

                points.append((
                    float(p_world[0]),
                    float(p_world[1]),
                    float(p_world[2])
                ))

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