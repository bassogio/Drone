#!/usr/bin/env python3
import math
import time
from unittest import result

from numpy import angle
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from drone_interfaces.action import DroneCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition

class MissionServer(Node):
    STATE_WAITING_FOR_POSITION = "WAITING_FOR_POSITION"
    STATE_GROUNDED = "GROUNDED"
    STATE_TAKEOFF = "TAKEOFF"
    STATE_HOVER = "HOVER"
    STATE_LAND = "LAND"
    STATE_MOVE = "MOVE"
    STATE_ERROR = "ERROR"
    
    def __init__(self):
        super().__init__('drone_mission_server')

        self.callback_group = ReentrantCallbackGroup()

        # ===============
        #  Parameters
        # ===============
        self.declare_parameter("target_x", float('nan'))
        self.declare_parameter("target_y", float('nan'))
        self.declare_parameter("target_z", float('nan'))
        self.declare_parameter("target_yaw", float('nan'))
        self.declare_parameter("state", self.STATE_WAITING_FOR_POSITION)

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None

        self.timer_period = 0.1
        self.position_threshold = 0.2
        self.yaw_threshold = 0.1 # radians (~5.7 degrees)
        self.takeoff_initial_height = -3.0

        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.target_z = self.get_parameter("target_z").value
        self.target_yaw = self.get_parameter("target_yaw").value
        self.state = self.get_parameter("state").value
        self.previous_state = self.state
        
        self.offboard_active = False
        self.takeoff_flag = False
        self.position_data_received = False
        
        # ===============
        #  Subscribers
        # ===============
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.pose_callback,
            qos_profile_sensor_data)

        # ===============
        #  Publishers
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
        #  Continuous control timer
        # ===============
        self.control_timer = self.create_timer(
            self.timer_period,
            self.control_loop,
            callback_group=self.callback_group)
    
        # ===============
        #  Action Server
        # ===============
        self.action_server = ActionServer(
            self,
            DroneCommand,
            'drone_command',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info('Mission action server is ready')
    
    def control_loop(self):
        if not self.offboard_active:
            return

        if None in [self.current_x, self.current_y, self.current_z, self.current_yaw]:
            return
        
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        # self.request_offboard_mode()
        # self.request_arm()
        
    def pose_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading
        
    def goal_callback(self, goal_request):
        """
        Called when a client sends a new goal request.

        This function does NOT perform the task.
        It only decides whether the server accepts or rejects the goal.
        """
        command    = goal_request.command
        target_x   = goal_request.target_x
        target_y   = goal_request.target_y
        target_z   = goal_request.target_z
        target_yaw = goal_request.target_yaw
        
        if math.isnan(target_x):
            target_x = self.current_x
        if math.isnan(target_y):
            target_y = self.current_y
        if math.isnan(target_z):
            target_z = self.current_z
        if math.isnan(target_yaw):
            target_yaw = self.current_yaw
            
        self.get_logger().info(
            f"Received goal request: "
            f"command={command.upper()}, "
            f"target x={target_x:.2f}, "
            f"target y={target_y:.2f}, "
            f"target z={target_z:.2f}, "
            f"target yaw={target_yaw:.2f}"
        )

        # ===============
        #  Validation rules
        # ===============
        allowed_commands = {"TAKEOFF", "HOVER", "LAND", "MOVE"}
        
        if goal_request.command == '':
            self.get_logger().warn('Rejecting goal because command is empty')
            return GoalResponse.REJECT
        
        if goal_request.command not in allowed_commands:
            self.get_logger().warn(
                f"Rejecting goal because command '{goal_request.command}' is not recognized."
            )
            return GoalResponse.REJECT
        
        # if goal_request.command == 'TAKEOFF' and not self.position_data_received:
        #     self.get_logger().warn('Rejecting takeoff goal because position data has not been received yet')
        #     return GoalResponse.REJECT
        
        if self.takeoff_flag and goal_request.command == 'TAKEOFF':
            self.get_logger().warn('Rejecting goal because drone has already taken off')
            return GoalResponse.REJECT
        
        if not self.takeoff_flag and goal_request.command != 'TAKEOFF':
            self.get_logger().warn('Rejecting goal because first command must be takeoff')
            return GoalResponse.REJECT
        
        # if target_z > 0:
        #     self.get_logger().warn('Rejecting goal because target_z must be non-positive (NED frame)')
        #     return GoalResponse.REJECT
        
        # Accept valid goals
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Called when the client requests cancellation.

        This only decides whether cancellation is allowed.
        The actual stop must still happen inside execute_callback().
        """

        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        This is the real worker function of the action server.
        The function is called when a new goal is accepted.
        1. Read the goal
        2. Start the task
        3. Publish feedback while running
        4. Watch for cancellation
        5. Return a final result
        """

        self.get_logger().info('Starting goal execution')

        # Read goal command value
        command = goal_handle.request.command

        # Create feedback and result message objects
        feedback_msg = DroneCommand.Feedback()
        result = DroneCommand.Result()

        # ===============
        # Task logic
        # ===============
        # Wait for position data if not received yet
        # Send feedback to inform the client that we are waiting for position data
        while None in [self.current_x, self.current_y, self.current_z, self.current_yaw]:
            if goal_handle.is_cancel_requested: # If the cancel was approved in cancel_callback, this will be True
                result = self.cancellation_requested(goal_handle, 
                                                     self.state,
                                                     self.STATE_GROUNDED,
                                                     reason='Goal was canceled before completion',
                                                     result=result)
                return result  
            
            self.set_state(self.state, self.STATE_WAITING_FOR_POSITION, reason="Waiting for position data")
            
            feedback_msg.state = self.state
            feedback_msg.current_x = float('nan')
            feedback_msg.current_y = float('nan')
            feedback_msg.current_z = float('nan')
            feedback_msg.current_yaw = float('nan')
            feedback_msg.distance_to_goal = float('nan')
            feedback_msg.yaw_error_to_goal = float('nan')
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(self.timer_period)
        
        if goal_handle.is_cancel_requested:
                # Tell ROS that this goal ended in the canceled state
                result = self.cancellation_requested(goal_handle, 
                                                     self.state,
                                                     self.STATE_HOVER,
                                                     reason='Goal was canceled by client before completion', 
                                                     result=result)
                # TODO: Should I add feedback info as well?
                return result            
        
        if command == 'TAKEOFF':
            self.takeoff_flag = True
            return self.takeoff_sequence(goal_handle, feedback_msg, result)

        # TODO: after while rclpy.ok() and when cancel should start landing
        if command == 'HOVER':
            return self.hover_sequence(goal_handle, feedback_msg, result)   
        
        if command == 'LAND':
            return self.land_sequence(goal_handle, feedback_msg, result)
        
        if command == 'MOVE':
            return self.move_sequence(goal_handle, feedback_msg, result)
        
        # if command == 'HOME':
        #     return self.home_sequence(goal_handle, feedback_msg, result)
        
        # if command == 'LIFT':
        #     return self.lift_sequence(goal_handle, feedback_msg, result)
        
        # if command == 'ROTATE':
        #     return self.rotate_sequence(goal_handle, feedback_msg, result)
        
        # if command == 'CIRCLE':
        #     return self.circle_sequence(goal_handle, feedback_msg, result)
        
    def set_state(self, previous_state, new_state, reason=None):
        if new_state != previous_state:
            if reason:
                self.get_logger().info(f"State: {previous_state} -> {new_state} | Reason: {reason}")
            else:
                self.get_logger().info(f"State: {previous_state} -> {new_state}")
        else:
            if reason:
                self.get_logger().info(f"State: {new_state} | Reason: {reason}")
            else:
                self.get_logger().info(f"State: {new_state}")
        
        self.state = new_state

    def cancellation_requested(self,
                               goal_handle,
                               previous_state, 
                               new_state, 
                               reason=None,
                               result: DroneCommand.Result = None):
        goal_handle.canceled()
        self.set_state(previous_state, new_state, reason)
        
        # Fill result object
        result.success = False
        if reason:
            result.message = reason
        else:
            result.message = "Goal was canceled"
        return result
    
    def successful_completion(self, 
                              goal_handle, 
                              previous_state, 
                              new_state, 
                              reason=None, 
                              result: DroneCommand.Result = None):
        goal_handle.succeed()
        self.set_state(previous_state, new_state, reason)
        
        # Fill result object
        result.success = True
        if reason:
            result.message = reason
        else:
            result.message = "Goal completed successfully"
        return result

    def hold_current_position(self):
        self.target_x = self.current_x
        self.target_y = self.current_y
        self.target_z = self.current_z
        self.target_yaw = self.current_yaw
        
    def takeoff_sequence(self, goal_handle, feedback_msg, result):
        self.takeoff_flag = True
        self.set_state(self.state, self.STATE_TAKEOFF, reason="Starting takeoff sequence")
        
        request = goal_handle.request
        self.target_x = self.current_x
        self.target_y = self.current_y
        self.target_z = request.target_z
        self.target_yaw = self.current_yaw
        
        if math.isnan(request.target_z):
            self.target_z = self.takeoff_initial_height
        
        # Publish setpoints for about 1 second before switching PX4 to offboard.
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            time.sleep(self.timer_period)
        
        self.request_offboard_mode()
        time.sleep(0.2)
        self.request_arm()
        self.offboard_active = True
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.hold_current_position()
                result = self.cancellation_requested(goal_handle, 
                                                     self.state,
                                                     self.STATE_HOVER,
                                                     reason="Takeoff cancled by client",
                                                     result=result)
                return result  

            distance = abs(self.current_z - self.target_z)
            self.publish_feedback(goal_handle, feedback_msg, state=self.state, distance=distance)

            if distance <= self.position_threshold:
                self.is_airborne = True
                # self.hold_current_position()
                return self.successful_completion(goal_handle,
                                                  self.state, 
                                                  self.STATE_HOVER, 
                                                  reason="Takeoff complete",
                                                  result=result)

            time.sleep(self.timer_period)

        goal_handle.abort()
        self.hold_current_position()
        self.set_state(self.state, self.STATE_HOVER, "Node stopped during takeoff")
        result.success = False
        result.message = "Node stopped during takeoff"
        return result               
    
    def hover_sequence(self, goal_handle, feedback_msg, result):
        self.set_state(self.state, self.STATE_HOVER, reason="Hovering in place")     
        
        self.target_x = self.current_x
        self.target_y = self.current_y
        self.target_z = self.current_z
        self.target_yaw = self.current_yaw
        
        # Publish setpoints for about 1 second before switching PX4 to offboard.
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            time.sleep(self.timer_period)
        
        self.request_offboard_mode()
        time.sleep(0.2)
        self.request_arm()
        self.offboard_active = True
        
        while rclpy.ok():
            self.publish_feedback(goal_handle, feedback_msg, state=self.state)
            
            if goal_handle.is_cancel_requested:
                self.hold_current_position()
                result = self.cancellation_requested(goal_handle, 
                                                     self.state,
                                                     self.STATE_LAND,
                                                     reason="Hovering cancled by client",
                                                     result=result)
            
        goal_handle.abort()
        self.set_state(self.state, self.STATE_ERROR, "Node stopped during hovering")
        result.success = False
        result.message = "Node stopped during hovering"
        return result      
                
    def land_sequence(self, goal_handle, feedback_msg, result):
        self.set_state(self.state, self.STATE_LAND, reason="Landing")     
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.hold_current_position()
                result = self.cancellation_requested(goal_handle, 
                                                     self.state,
                                                     self.STATE_HOVER,
                                                     reason="Takeoff cancled by client",
                                                     result=result)
                if altitude_from_home > self.position_threshold:
                    self.takeoff_flag = True
                return result  
            
            altitude_from_home = abs(self.current_z)
            self.publish_feedback(goal_handle, feedback_msg, self.state, altitude_from_home)

            if altitude_from_home <= self.position_threshold:
                self.takeoff_flag = False
                self.offboard_active = False

                self.successful_completion(goal_handle,
                                           self.state, 
                                           self.STATE_GROUNDED, 
                                           reason="Landing complete",
                                           result=result)
                return result

            time.sleep(self.timer_period)

        goal_handle.abort()
        self.hold_current_position()
        self.set_state(self.state, self.STATE_HOVER, "Node stopped while landing")
        result.success = False
        result.message = "Node stopped while landing"
        return result       
         
    def move_sequence(self, goal_handle, feedback_msg, result):
        self.set_state(self.state, self.STATE_MOVE, reason="Moving to target position")     
        
        request = goal_handle.request

        # Detect what the user actually requested BEFORE replacing NaN
        x_requested = not math.isnan(request.target_x)
        y_requested = not math.isnan(request.target_y)
        z_requested = not math.isnan(request.target_z)
        yaw_requested = not math.isnan(request.target_yaw)

        position_requested = x_requested or y_requested or z_requested
        yaw_only_move = (not position_requested) and yaw_requested

        self.target_x = request.target_x
        self.target_y = request.target_y
        self.target_z = request.target_z
        self.target_yaw = request.target_yaw
        
        if math.isnan(self.target_x):
            self.target_x = self.current_x
        if math.isnan(self.target_y):
            self.target_y = self.current_y
        if math.isnan(self.target_z):
            self.target_z = self.current_z

        if math.isnan(self.target_yaw):
            self.target_yaw = self.current_yaw
        else:
            # If this MOVE is yaw only, treat yaw as relative
            if yaw_only_move:
                self.target_yaw = self.normalize_angle(self.current_yaw + self.target_yaw)
            else:
                # Normal MOVE with position uses absolute yaw
                self.target_yaw = self.normalize_angle(self.target_yaw)
            
        # Publish setpoints for about 1 second before switching PX4 to offboard.
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            time.sleep(self.timer_period)
        
        self.request_offboard_mode()
        time.sleep(0.2)
        self.request_arm()
        self.offboard_active = True
        
        while rclpy.ok():
            distance = self.distance_to_target()
            yaw_error = abs(self.yaw_error_to_target())
            
            self.publish_feedback(goal_handle,
                                feedback_msg,
                                state=self.state,
                                distance=distance,
                                yaw_error=yaw_error)
            
            if goal_handle.is_cancel_requested:
                self.hold_current_position()
                result = self.cancellation_requested(goal_handle, 
                                                    self.state,
                                                    self.STATE_HOVER,
                                                    reason="Move cancled by client",
                                                    result=result)
                return result
            
            if yaw_only_move:
                position_ok = True
                yaw_ok = yaw_error <= self.yaw_threshold
            else:
                position_ok = distance <= self.position_threshold
                yaw_ok = yaw_error <= self.yaw_threshold
        
            if position_ok and yaw_ok:
                self.is_airborne = True
                self.hold_current_position()
                return self.successful_completion(goal_handle,
                                                self.state, 
                                                self.STATE_HOVER, 
                                                reason="Move complete, hovering",
                                                result=result)
                
            time.sleep(self.timer_period)
                            
        goal_handle.abort()
        self.hold_current_position()
        self.set_state(self.state, self.STATE_HOVER, "Node stopped while moving to target")
        result.success = False
        result.message = "Node stopped while moving to target"
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
        
    def yaw_error_to_target(self):
        error = self.target_yaw - self.current_yaw
        return math.atan2(math.sin(error), math.cos(error))
    
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_feedback(self, goal_handle, feedback_msg, state, distance=None, yaw_error=None):
        feedback_msg.state = state
        feedback_msg.current_x = float(self.current_x) if self.current_x is not None else 0.0
        feedback_msg.current_y = float(self.current_y) if self.current_y is not None else 0.0
        feedback_msg.current_z = float(self.current_z) if self.current_z is not None else 0.0
        feedback_msg.current_yaw = float(self.current_yaw) if self.current_yaw is not None else 0.0
        if distance is not None:
            feedback_msg.distance_to_goal = float(distance)  
        if yaw_error is not None:
            feedback_msg.yaw_error_to_goal = float(yaw_error)          
        goal_handle.publish_feedback(feedback_msg)
        
    def destroy(self):
        """
        Clean shutdown helper.
        """
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MissionServer()

    # MultiThreadedExecutor is useful for action servers, especially
    # when execute_callback is busy and cancellation still needs to be handled. :contentReference[oaicite:3]{index=3}
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()