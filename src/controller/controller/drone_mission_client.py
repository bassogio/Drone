#!/usr/bin/env python3

import math
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from drone_interfaces.action import DroneCommand

@dataclass
class MissionStep:
    step_id: int 
    name: str = ""
    command: str = ""

    target_x: float = math.nan
    target_y: float = math.nan
    target_z: float = math.nan
    target_yaw: float = math.nan

class MissionClient(Node):
    def __init__(self):
        super().__init__('drone_mission_client')

        # ===============
        #  Action Client
        # ===============
        # Create the action client.
        self.action_client = ActionClient(
            self,
            DroneCommand,
            'drone_command' # The action name must match the server's action name exactly.
        )  

        # Keep references to futures and goal handle
        self._send_goal_future = None
        self._get_result_future = None
        self.goal_handle: ClientGoalHandle | None = None

        # self.missions = [
        #     MissionStep(step_id=1, name="Takeoff", command="TAKEOFF", target_z=5.0),
        #     MissionStep(step_id=2, name="Move to A", command="MOVE", target_x=1.0),
        #     MissionStep(step_id=3, name="Move to B", command="MOVE", target_x=8.0, target_y=3.0, target_z=5.0),
        #     MissionStep(step_id=4, name="Land", command="LAND"),
        # ]
        self.missions = [
            MissionStep(step_id=1, name="Takeoff", command="TAKEOFF", target_z=5.0),
            MissionStep(step_id=2, name="Spin +30", command="MOVE", target_yaw=30.0),
            MissionStep(step_id=3, name="Spin -30", command="MOVE", target_yaw=-30.0),
            MissionStep(step_id=4, name="Spin +90", command="MOVE", target_yaw=90.0),
            MissionStep(step_id=5, name="Spin -90", command="MOVE", target_yaw=-90.0),
            MissionStep(step_id=6, name="Land", command="LAND"),
        ]   
        self.current_mission_index = 0
        
        self.get_logger().info('Mission client created')
        
    def send_goal(self, mission):
        """
        Send a goal to the server.

        This is the main method you call to start an action.
        """
        if self.current_mission_index == 0:
            self.get_logger().info('Waiting for action server...')
            self.action_client.wait_for_server()
            self.get_logger().info('Action server found')

        mission = self.missions[self.current_mission_index]
        self.get_logger().info(f"Starting mission step {mission.step_id}: {mission.name}")
        
        # Create goal message
        goal_msg = DroneCommand.Goal()
        goal_msg.command = mission.command
        goal_msg.target_x = mission.target_x
        goal_msg.target_y = mission.target_y
        goal_msg.target_z = -mission.target_z
        goal_msg.target_yaw = mission.target_yaw * math.pi / 180.0 # Convert degrees to radians

        # Send the goal asynchronously
        # feedback_callback is called every time the server publishes feedback
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # When the server replies "accepted" or "rejected",
        # this callback will be triggered
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        Called every time the server sends feedback.
        """

        fb = feedback_msg.feedback

        self.get_logger().info(
            f"state={fb.state} | "
            f"x={fb.current_x:.2f}, y={fb.current_y:.2f}, z={fb.current_z:.2f}, "
            f"yaw={fb.current_yaw:.2f}, dist={fb.distance_to_goal:.2f}, yaw_err={fb.yaw_error_to_goal:.2f}"
        )

        # =========================
        # TODO: Optional cancel trigger
        # =========================
        #
        # Example:
        # if feedback.progress >= 50.0:
        #     self.cancel_goal()
        #
        # Leave commented unless you actually want automatic cancel behavior.

    def goal_response_callback(self, future):
        """
        Called when the server accepts or rejects the goal.
        """

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the server')
            return

        self.get_logger().info('Goal was accepted by the server')

        # After acceptance, request the final result asynchronously
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Called once when the server sends the final result.
        """

        result_wrapper = future.result()
        result = result_wrapper.result
        status = result_wrapper.status
        
        self.get_logger().info('Final result received:')
        self.get_logger().info(f'  status  = {status}')
        self.get_logger().info(f'  success = {result.success}')
        self.get_logger().info(f'  message = "{result.message}"')
        
        if result.success:
            self.current_mission_index += 1
            self.send_goal(self.missions[self.current_mission_index]) # Start next mission step
        else:
            self.get_logger().error('Mission failed. Not proceeding to next step.')

    def result_callback(self, future):
        result_wrapper = future.result()
        result = result_wrapper.result
        status = result_wrapper.status

        status_name = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, f"UNRECOGNIZED({status})")

        message = result.message if result.message else "<empty message from server>"

        self.get_logger().info('Final result received:')
        self.get_logger().info(f'  status  = {status} ({status_name})')
        self.get_logger().info(f'  success = {result.success}')
        self.get_logger().info(f'  message = "{message}"')

        if status == GoalStatus.STATUS_SUCCEEDED and not result.success:
            self.get_logger().warn(
                'ROS action status says SUCCEEDED, but result.success is False. '
                'This means the server logic is inconsistent.'
            )

        if result.success:
            self.current_mission_index += 1

            if self.current_mission_index < len(self.missions):
                self.send_goal(self.missions[self.current_mission_index])
            else:
                self.get_logger().info('All mission steps completed')
        else:
            self.get_logger().error('Mission failed. Not proceeding to next step.')
    
    def cancel_goal(self):
        """
        Request cancellation of the active goal.
        """

        if self.goal_handle is None:
            self.get_logger().warn('No active goal handle. Cannot cancel.')
            return

        self.get_logger().warn('Sending cancel request')
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """
        Called when ROS answers the cancel request itself.
        """

        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancel request accepted')
        else:
            self.get_logger().warn('Cancel request rejected')

    def start_mission(self):
        if self.current_mission_index < len(self.missions):
            self.send_goal(self.missions[self.current_mission_index])
        else:
            self.get_logger().info('All mission steps completed')
            
def main(args=None):
    rclpy.init(args=args)

    node = MissionClient()

    node.start_mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()