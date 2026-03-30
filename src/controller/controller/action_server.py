#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition

from drone_interfaces.action import DroneCommand

class DroneActionServer(Node):
    def __init__(self):
        super().__init__("drone_action_server")

        self.callback_group = ReentrantCallbackGroup()

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        self.position_threshold = 0.20
        self.timer_period = 0.1

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10
        )

        self._action_server = ActionServer(
            self,
            DroneCommand,
            "drone_command",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("Drone action server started")

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
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0
    ):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

    def distance_to_target(self):
        if None in [self.current_x, self.current_y, self.current_z]:
            return float("inf")
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dz = self.target_z - self.current_z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

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
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0
    ):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

    def distance_to_target(self):
        if None in [self.current_x, self.current_y, self.current_z]:
            return float("inf")
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dz = self.target_z - self.current_z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def goal_callback(self, goal_request):
        command = goal_request.command.upper()
        allowed = {"TAKEOFF", "MOVE", "LAND", "HOVER"}

        if command not in allowed:
            self.get_logger().warn(f"Rejected goal with unsupported command: {command}")
            return GoalResponse.REJECT

        self.get_logger().info(f"Accepted goal: {command}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        command = goal.command.upper()

        feedback_msg = DroneCommand.Feedback()
        result = DroneCommand.Result()

        while None in [self.current_x, self.current_y, self.current_z, self.current_yaw]:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled while waiting for position data"
                return result

            feedback_msg.state = "WAITING_FOR_POSITION"
            feedback_msg.current_x = 0.0
            feedback_msg.current_y = 0.0
            feedback_msg.current_z = 0.0
            feedback_msg.current_yaw = 0.0
            feedback_msg.distance_to_goal = -1.0
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        if command == "TAKEOFF":
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = goal.z if goal.z != 0.0 else -3.0
            self.target_yaw = goal.yaw

            for _ in range(10):
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                time.sleep(0.1)

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Takeoff canceled"
                    return result

                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()

                distance = self.distance_to_target()

                feedback_msg.state = "TAKEOFF"
                feedback_msg.current_x = float(self.current_x)
                feedback_msg.current_y = float(self.current_y)
                feedback_msg.current_z = float(self.current_z)
                feedback_msg.current_yaw = float(self.current_yaw)
                feedback_msg.distance_to_goal = float(distance)
                goal_handle.publish_feedback(feedback_msg)

                if abs(self.current_z - self.target_z) <= self.position_threshold:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Takeoff complete"
                    return result

                time.sleep(self.timer_period)

        elif command == "MOVE":
            self.target_x = goal.x
            self.target_y = goal.y
            self.target_z = goal.z
            self.target_yaw = goal.yaw

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Move canceled"
                    return result

                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()

                distance = self.distance_to_target()

                feedback_msg.state = "MOVE"
                feedback_msg.current_x = float(self.current_x)
                feedback_msg.current_y = float(self.current_y)
                feedback_msg.current_z = float(self.current_z)
                feedback_msg.current_yaw = float(self.current_yaw)
                feedback_msg.distance_to_goal = float(distance)
                goal_handle.publish_feedback(feedback_msg)

                if distance <= self.position_threshold:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Move complete"
                    return result

                time.sleep(self.timer_period)

        elif command == "HOVER":
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.target_yaw = self.current_yaw

            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

            goal_handle.succeed()
            result.success = True
            result.message = "Hover setpoint published"
            return result

        elif command == "LAND":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            goal_handle.succeed()
            result.success = True
            result.message = "Land command sent"
            return result

        goal_handle.abort()
        result.success = False
        result.message = f"Unhandled command: {command}"
        return result

def main(args=None):
    rclpy.init(args=args)

    node = DroneActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()