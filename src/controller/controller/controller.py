#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleLocalPosition

class ControllerNode(Node): 
    def __init__(self):
        super().__init__("my_controller")  

        # ===============
        #   Parameters 
        # ===============
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.0)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("state", None)  # e.g., "IDLE", "TAKEOFF", "LAND", "HOVER", "MOVE"
        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.target_z = self.get_parameter("target_z").value
        self.target_yaw = self.get_parameter("target_yaw").value
        self.state = self.get_parameter("state").value
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        timer_perios = 0.1  # seconds

        # ===============
        #   Subscribers
        # ===============
        self.position = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback, 10)

        # ===============
        #   Publishers
        # ===============
        self.offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10)

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10)
        
        #  Timer 
        self.timer_ = self.create_timer(timer_perios, self.timer_callback)
        self.get_logger().info("Node started")

    def timer_callback(self):
        if None in [self.current_x, self.current_y, self.current_z, self.current_yaw]:
            self.get_logger().info("Waiting for position data...")
            return

        self.get_logger().info(f"x={self.current_x:.3f} | y={self.current_y:.3f} | z={self.current_z:.3f} | yaw={self.current_yaw:.3f}")

        if self.state == "TAKEOFF":
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = -5.0  # Takeoff to 5 meters altitude
            self.target_yaw = 0.0
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
        
        if self.state == "MOVE":
            self.target_x = self.current_x + 5.0  # Move 5 meters forward
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.target_yaw = 0.0
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_mode.publish(msg)
    
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()