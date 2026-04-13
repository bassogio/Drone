#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry


class PX4toROS2converter(Node):
    def __init__(self):
        super().__init__('px4_to_ros2_converter')

        # ===============
        #  Parameters
        # ===============
        self.declare_parameter('px4_odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('ros_odom_topic', '/drone/vehicle_odometry')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.px4_odom_topic = self.get_parameter('px4_odom_topic').value
        self.ros_odom_topic = self.get_parameter('ros_odom_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ===============
        #  Subscribers
        # ===============
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            self.px4_odom_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )

        # ===============
        #  Publishers
        # ===============
        self.odom_pub = self.create_publisher(
            Odometry, 
            self.ros_odom_topic, 
            10)

        self.get_logger().info(f'Subscribed to: {self.px4_odom_topic}')
        self.get_logger().info(f'Publishing ROS odometry on: {self.ros_odom_topic}')

    def odom_callback(self, msg: VehicleOdometry):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warning(
                f'Unsupported pose_frame: {msg.pose_frame}. This simple node only supports POSE_FRAME_NED.'
            )
            return

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(msg.position[1])
        odom.pose.pose.position.y = float(msg.position[0])
        odom.pose.pose.position.z = float(-msg.position[2])

        odom.pose.pose.orientation.x = float(msg.q[1])
        odom.pose.pose.orientation.y = float(msg.q[2])
        odom.pose.pose.orientation.z = float(msg.q[3])
        odom.pose.pose.orientation.w = float(msg.q[0])

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = PX4toROS2converter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()