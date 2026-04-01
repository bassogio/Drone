#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from drone_interfaces.action import DroneCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition


class DroneActionServer(Node):
    def __init__(self):
        super().__init__('drone_action_server')

        self.callback_group = ReentrantCallbackGroup()

        # ===============
        #   Parameters
        # ===============
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.0)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("state", "OFF")

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.timer_period = 0.1
        self.position_threshold = 0.1
        self.takeoff_initial_height = -3.0

        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.target_z = self.get_parameter("target_z").value
        self.target_yaw = self.get_parameter("target_yaw").value
        self.state = self.get_parameter("state").value

        self.offboard_active = False
        
        self.takeoff_flag = False

        # ===============
        #   Subscribers
        # ===============
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback,
            qos_profile_sensor_data)

        # ===============
        #   Publishers
        # ===============
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10)

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10)

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10)

        # ===============
        #   Continuous control timer
        # ===============
        self.control_timer = self.create_timer(
            self.timer_period,
            self.control_loop,
            callback_group=self.callback_group)

        # ===============
        #   Action Server
        # ===============
        self._action_server = ActionServer(
            self,
            DroneCommand,
            'drone_command',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group)

        self.get_logger().info("Drone action server started")

    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    def control_loop(self):
        if not self.offboard_active:
            return

        if None in [self.current_x, self.current_y, self.current_z, self.current_yaw]:
            return

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    def goal_callback(self, goal_request: DroneCommand):
        self.get_logger().info("Received a goal")

        command = goal_request.command.upper()
        allowed_commands = {"OFF", "TAKEOFF", "HOVER", "LAND", "MOVE"}

        if command not in allowed_commands:
            self.get_logger().warn(f"Goal rejected, unsupported command: {command}")
            return GoalResponse.REJECT
        
        if not self.takeoff_flag and not command == "TAKEOFF":
            self.get_logger().warn(f"Goal rejected, Drone should first takeoff")
            return GoalResponse.REJECT

        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        command = goal_handle.request.command.upper()
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
            time.sleep(self.timer_period)

        # TODO: I should add HOME, LIFT, ROTATE, yaw feedback and not only distance
        if command == "TAKEOFF":
            target_z = goal_handle.request.target_z

            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = target_z if not math.isnan(target_z) else self.takeoff_initial_height
            self.target_yaw = self.current_yaw
            self.state = "TAKEOFF"
            self.offboard_active = True

            # Let the timer publish Offboard heartbeat and setpoints for about 1 second
            for _ in range(10):
                time.sleep(self.timer_period)

            self.request_offboard_mode()
            self.request_arm()

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.target_x = self.current_x
                    self.target_y = self.current_y
                    self.target_z = self.current_z
                    self.target_yaw = self.current_yaw
                    self.state = "HOVER"

                    goal_handle.canceled()
                    result.success = False
                    result.message = "Takeoff canceled, hovering at current position"
                    return result

                distance = self.distance_to_target()

                feedback_msg.state = "TAKEOFF"
                feedback_msg.current_x = float(self.current_x)
                feedback_msg.current_y = float(self.current_y)
                feedback_msg.current_z = float(self.current_z)
                feedback_msg.current_yaw = float(self.current_yaw)
                feedback_msg.distance_to_goal = float(distance)
                goal_handle.publish_feedback(feedback_msg)

                if abs(self.current_z - self.target_z) <= self.position_threshold:
                    self.state = "HOVER"
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Takeoff complete, hovering"
                    self.takeoff_flag = True
                    return result

                time.sleep(self.timer_period)
        
        elif command == "HOVER":
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.target_yaw = self.current_yaw
            self.state = "HOVER"
            self.offboard_active = True

            goal_handle.succeed()
            result.success = True
            result.message = "Hovering at current position"
            return result

        elif command == "LAND":
            self.state = "LAND"
            self.offboard_active = False
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            goal_handle.succeed()
            result.success = True
            result.message = "Land command sent"
            self.takeoff_flag = False
            return result

        elif command == "MOVE":
            self.target_x = goal_handle.request.target_x

            if not math.isnan(goal_handle.request.target_y):
                self.target_y = goal_handle.request.target_y
            else:
                self.target_y = self.current_y

            if not math.isnan(goal_handle.request.target_z):
                self.target_z = goal_handle.request.target_z
            else:
                self.target_z = self.current_z

            if not math.isnan(goal_handle.request.target_yaw):
                self.target_yaw = goal_handle.request.target_yaw
            else:
                self.target_yaw = self.current_yaw
                                
            self.state = "MOVE"
            self.offboard_active = True

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.target_x = self.current_x
                    self.target_y = self.current_y
                    self.target_z = self.current_z
                    self.target_yaw = self.current_yaw
                    self.state = "HOVER"

                    goal_handle.canceled()
                    result.success = False
                    result.message = "Move canceled, hovering at current position"
                    return result

                distance = self.distance_to_target()

                feedback_msg.state = "MOVE"
                feedback_msg.current_x = float(self.current_x)
                feedback_msg.current_y = float(self.current_y)
                feedback_msg.current_z = float(self.current_z)
                feedback_msg.current_yaw = float(self.current_yaw)
                feedback_msg.distance_to_goal = float(distance)
                goal_handle.publish_feedback(feedback_msg)

                if distance <= self.position_threshold:
                    self.state = "HOVER"
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Move complete, hovering"
                    return result

                time.sleep(self.timer_period)

        elif command == "OFF":
            self.state = "OFF"
            self.offboard_active = False

            goal_handle.succeed()
            result.success = True
            result.message = "Offboard publishing stopped"
            return result

        goal_handle.abort()
        result.success = False
        result.message = f"Unhandled command: {command}"
        return result

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
        msg.position = [float(self.target_x), float(self.target_y), float(self.target_z)]
        msg.yaw = float(self.target_yaw)
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

    def request_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

    def request_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

    def distance_to_target(self):
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dz = self.target_z - self.current_z
        return math.sqrt(dx**2 + dy**2 + dz**2)


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


if __name__ == '__main__':
    main()