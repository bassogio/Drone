#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data

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
        self.declare_parameter("state", "ON")  # e.g., "IDLE", "TAKEOFF", "LAND", "HOVER", "MOVE"
        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.target_z = self.get_parameter("target_z").value
        self.target_yaw = self.get_parameter("target_yaw").value
        self.state = self.get_parameter("state").value
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        self.timer_period = 0.1  # seconds
        
        # Target threshold
        self.z_target_threshold = 0.2  # meters

        # Flags
        self.arming_state = None
        self.nav_state = None
        self.takeoff_initiated = False
        self.land_initiated = False

        # ===============
        #   Subscribers
        # ===============
        self.position = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback, 
            qos_profile_sensor_data)
        
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
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )
        
        # ===============
        #   Timer
        # ===============
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Node started")

    # Callback function to update current position and yaw from the vehicle_local_position topic
    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    # Publish control mode to enable position control, and disable all other modes
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
    
    # Publish trajectory setpoint with the target position and yaw
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_pub.publish(msg)

    # Publish a vehicle command, e.g., to arm, disarm, takeoff, land, etc.
    def publish_vehicle_command(self, command, 
                                param1=0.0, 
                                param2=0.0,
                                param3=0.0,
                                param4=0.0,
                                param5=0.0,
                                param6=0.0,
                                param7=0.0):
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

    def timer_callback(self):

        if None in [self.current_x, self.current_y, self.current_z, self.current_yaw]:
            self.get_logger().info("Waiting for position data...")
            return
        
        # self.get_logger().info(f"x={self.current_x:.3f} | y={self.current_y:.3f} | z={self.current_z:.3f} | yaw={self.current_yaw:.3f}")
        
        self.state = self.get_parameter("state").value

        if self.state == "TAKEOFF":
            if not self.takeoff_initiated:
                self.get_logger().info("Taking off...")
            self.takeoff_initiated = True

            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = -5.0  # Takeoff to 5 meters altitude
            self.target_yaw = 0.0

            self.publish_offboard_control_mode() # Enable offboard control mode
            self.publish_trajectory_setpoint() # Publish the takeoff setpoint

            # Set mode to offboard mode - this is required to start accepting offboard control commands
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )
            # Arm the drone - this is required to start accepting offboard control commands
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )
            self.state = "WAIT_FOR_TAKEOFF_COMPLETE"
        
        if self.state == "WAIT_FOR_TAKEOFF_COMPLETE":
            # Wait until the drone reaches the takeoff altitude
            if abs(self.current_z - self.target_z) <= self.z_target_threshold:
                self.get_logger().info("Takeoff complete!")
                self.state = "HOVER"
        
        if self.state == "HOVER":
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z 
            self.target_yaw = self.current_yaw

            self.publish_offboard_control_mode() # Enable offboard control mode
            self.publish_trajectory_setpoint() # Publish the takeoff setpoint

            # Set mode to offboard mode - this is required to start accepting offboard control commands
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )
            # Arm the drone - this is required to start accepting offboard control commands
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()