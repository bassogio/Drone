#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        self.offboard_setpoint_counter = 0

        # PX4 uses NED:
        # x = North, y = East, z = Down
        # So to go UP by 3 meters, use z = -3.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -3.0
        self.target_yaw = 0.0
        
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # 10 Hz loop
        self.timer = self.create_timer(0.1, self.timer_callback)

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
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        # Keep publishing offboard control mode and the target setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # After a few setpoints, switch to Offboard and arm
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0
            )

            self.get_logger().info('Sent Offboard mode and arm command')

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()