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
        self.declare_parameter("state", "OFF")  # "ON", "TAKEOFF", "LAND", "HOVER", "MOVE"
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        self.timer_period = 0.1  # seconds

        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.target_z = self.get_parameter("target_z").value
        self.target_yaw = self.get_parameter("target_yaw").value
        self.state = self.get_parameter("state").value
        
        self.takeoff_initial_height = -3.0
        
        # Target threshold
        self.position_threshold = 0.2  # meters
        
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
            10)
                
        # ===============
        #   Server
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

    # Callback function to update current position and yaw from the vehicle_local_position topic
    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading          
                
    def goal_callback(self, goal_request: DroneCommand):
        self.get_logger().info("Received a goal")
        
        command = goal_request.command
        allowed_commands = {"OFF", "ON", "TAKEOFF", "HOVER", "LAND", "MOVE"}
        
        if command not in allowed_commands:
            self.get_logger().warn(f"Goal rejected - unsupported command: {command}")
            return GoalResponse.REJECT
        # TODO: Think of more reasons to reject a command. For example, target_z < 0
        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        command = goal_handle.request.command
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        target_z = goal_handle.request.target_z
        target_yaw = goal_handle.request.target_yaw
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
        
        if command == "ON":
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.target_yaw = self.current_yaw
    
        elif command == "TAKEOFF":
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = target_z if target_z != 0.0 else self.takeoff_initial_height
            self.target_yaw = self.current_yaw

            self.publish_offboard_control_mode() # Enable offboard control mode
            self.publish_trajectory_setpoint() # Publish the takeoff setpoint

            self.request_offboard_mode() # Set mode to offboard mode
            self.request_arm() # Arm the drone 
            
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Move canceled"
                    return result
                
                distance = self.distance_to_target()
                
                feedback_msg.state = "TAKEOFF"
                feedback_msg.current_x = self.current_x
                feedback_msg.current_y = self.current_y
                feedback_msg.current_z = self.current_z
                feedback_msg.current_yaw = self.current_yaw
                feedback_msg.distance_to_goal = distance
                goal_handle.publish_feedback(feedback_msg)
                
                if abs(self.current_z - self.target_z) <= self.position_threshold:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Takeoff complete"
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
        
        elif command == "MOVE":
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
  
        goal_handle.abort()
        result.success = False
        result.message = f"Unhandled command: {command}"
        return result
    
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
        
    # Set mode to offboard mode - this is required to start accepting offboard control commands
    def request_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )
        self.offboard_requested = True

    # Arm the drone - this is required to start accepting offboard control commands
    def request_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )
        self.arm_requested = True      

    def distance_to_target(self):
        dx = abs(self.target_x - self.current_x)
        dy = abs(self.target_y - self.current_y)
        dz = abs(self.target_z - self.current_z)
        return math.sqrt(dx**2 + dy**2 + dz**2)
        
        
def main(args=None):
    rclpy.init(args=args)
    drone_action_server = DroneActionServer()
    rclpy.spin(drone_action_server)
    drone_action_server.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()