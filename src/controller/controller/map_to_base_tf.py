#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from rclpy.qos import qos_profile_sensor_data


class MapToBaseTF(Node):
    def __init__(self):
        super().__init__('map_to_base_tf')

        self.br = TransformBroadcaster(self)

        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # PX4 quaternion order is [w, x, y, z]
        self.q = [1.0, 0.0, 0.0, 0.0]

        # Flags so we know when data arrived
        self.have_pose = False
        self.have_attitude = False

        self.att_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos_profile_sensor_data
        )

        self.pose_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback,
            qos_profile_sensor_data
        )

        # Publish TF at fixed rate
        self.timer = self.create_timer(0.05, self.tf_callback)

        self.get_logger().info('Publishing dynamic transform: map -> base_link')

    def pose_callback(self, msg):
        self.current_x = float(msg.x)
        self.current_y = float(msg.y)
        self.current_z = float(msg.z)
        self.have_pose = True

    def attitude_callback(self, msg):
        # Convert all values to plain Python float
        self.q = [float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])]
        self.have_attitude = True

    def tf_callback(self):
        if not self.have_pose or not self.have_attitude:
            return

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_link'

        # Position
        t.transform.translation.x = float(self.current_x)
        t.transform.translation.y = float(self.current_y)
        t.transform.translation.z = float(self.current_z)

        # Rotation
        # PX4 gives [w, x, y, z]
        # geometry_msgs wants x, y, z, w
        t.transform.rotation.x = float(self.q[1])
        t.transform.rotation.y = float(self.q[2])
        t.transform.rotation.z = float(self.q[3])
        t.transform.rotation.w = float(self.q[0])

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapToBaseTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()