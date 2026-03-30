#!/usr/bin/env python3
import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition


class FlightPhase(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    HOLD = auto()
    MOVE = auto()
    LANDING = auto()
    FAILSAFE = auto()


class ControllerNode(Node):
    def __init__(self):
        super().__init__("my_controller")

        # Parameters
        self.declare_parameter("state", "IDLE")
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", -3.0)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("takeoff_height_m", 3.0)
        self.declare_parameter("position_tolerance_m", 0.25)
        self.declare_parameter("timer_period_s", 0.1)
        self.declare_parameter("offboard_warmup_cycles", 10)

        # Internal controller state
        self.phase = FlightPhase.IDLE
        self.last_logged_phase = None
        self.offboard_warmup_counter = 0

        # Internal command flags
        self.offboard_requested = False
        self.arm_requested = False
        self.land_requested = False

        # Vehicle state feedback
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        # Hold / target memory
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0
        self.hold_yaw = 0.0

        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback,
            qos_profile_sensor_data,
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        # Timer
        timer_period = self.get_parameter("timer_period_s").value
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Controller node started")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    # -----------------------------
    # Helper methods
    # -----------------------------
    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def log_phase_once(self, text: str):
        if self.last_logged_phase != text:
            self.get_logger().info(text)
            self.last_logged_phase = text

    def has_position(self) -> bool:
        return None not in [self.current_x, self.current_y, self.current_z, self.current_yaw]

    def distance_to_target(self, x: float, y: float, z: float) -> float:
        return math.sqrt(
            (self.current_x - x) ** 2 +
            (self.current_y - y) ** 2 +
            (self.current_z - z) ** 2
        )

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = self.now_us()
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self.now_us()
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
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
        msg.timestamp = self.now_us()
        self.vehicle_command_pub.publish(msg)

    def request_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )
        self.offboard_requested = True

    def request_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )
        self.arm_requested = True

    def request_disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0
        )

    def request_land(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.land_requested = True

    def reset_command_flags(self):
        self.offboard_requested = False
        self.arm_requested = False
        self.land_requested = False

    # -----------------------------
    # State handlers
    # -----------------------------
    def handle_idle(self):
        self.log_phase_once("Phase: IDLE")

        self.hold_x = self.current_x
        self.hold_y = self.current_y
        self.hold_z = self.current_z
        self.hold_yaw = self.current_yaw

        self.offboard_warmup_counter = 0
        self.reset_command_flags()

        requested = self.get_parameter("state").value

        if requested == "TAKEOFF":
            self.phase = FlightPhase.TAKEOFF
        elif requested == "MOVE":
            self.phase = FlightPhase.MOVE
        elif requested == "LAND":
            self.phase = FlightPhase.LANDING

    def handle_takeoff(self):
        self.log_phase_once("Phase: TAKEOFF")

        takeoff_height = abs(self.get_parameter("takeoff_height_m").value)
        target_z = self.hold_z - takeoff_height

        # Continuously publish offboard heartbeat and setpoint
        self.publish_offboard_control_mode()
        self.publish_position_setpoint(
            self.hold_x,
            self.hold_y,
            target_z,
            self.hold_yaw
        )

        warmup_cycles = int(self.get_parameter("offboard_warmup_cycles").value)

        if self.offboard_warmup_counter < warmup_cycles:
            self.offboard_warmup_counter += 1
            return

        if not self.offboard_requested:
            self.get_logger().info("Requesting OFFBOARD mode")
            self.request_offboard_mode()

        if not self.arm_requested:
            self.get_logger().info("Requesting ARM")
            self.request_arm()

        tolerance = self.get_parameter("position_tolerance_m").value
        if self.distance_to_target(self.hold_x, self.hold_y, target_z) < tolerance:
            self.hold_z = target_z
            self.phase = FlightPhase.HOLD

    def handle_hold(self):
        self.log_phase_once("Phase: HOLD")

        self.publish_offboard_control_mode()
        self.publish_position_setpoint(
            self.hold_x,
            self.hold_y,
            self.hold_z,
            self.hold_yaw
        )

        requested = self.get_parameter("state").value

        if requested == "MOVE":
            self.phase = FlightPhase.MOVE
        elif requested == "LAND":
            self.phase = FlightPhase.LANDING

    def handle_move(self):
        self.log_phase_once("Phase: MOVE")

        target_x = self.get_parameter("target_x").value
        target_y = self.get_parameter("target_y").value
        target_z = self.get_parameter("target_z").value
        target_yaw = self.get_parameter("target_yaw").value

        self.publish_offboard_control_mode()
        self.publish_position_setpoint(target_x, target_y, target_z, target_yaw)

        self.hold_x = target_x
        self.hold_y = target_y
        self.hold_z = target_z
        self.hold_yaw = target_yaw

        requested = self.get_parameter("state").value

        if requested == "HOLD":
            self.phase = FlightPhase.HOLD
        elif requested == "LAND":
            self.phase = FlightPhase.LANDING

    def handle_landing(self):
        self.log_phase_once("Phase: LANDING")

        if not self.land_requested:
            self.get_logger().info("Requesting LAND")
            self.request_land()

    def handle_failsafe(self):
        self.log_phase_once("Phase: FAILSAFE")

        if not self.land_requested:
            self.get_logger().info("FAILSAFE: requesting LAND")
            self.request_land()

    # -----------------------------
    # Main loop
    # -----------------------------
    def timer_callback(self):
        if not self.has_position():
            self.get_logger().info("Waiting for position data...")
            return

        if self.phase == FlightPhase.IDLE:
            self.handle_idle()
        elif self.phase == FlightPhase.TAKEOFF:
            self.handle_takeoff()
        elif self.phase == FlightPhase.HOLD:
            self.handle_hold()
        elif self.phase == FlightPhase.MOVE:
            self.handle_move()
        elif self.phase == FlightPhase.LANDING:
            self.handle_landing()
        elif self.phase == FlightPhase.FAILSAFE:
            self.handle_failsafe()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()