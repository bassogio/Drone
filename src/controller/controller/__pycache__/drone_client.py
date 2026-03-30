#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil

class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__('count_until_client')

        # ActionClient(node, ActionType, action_name)
        self.action_client_ = ActionClient(self, CountUntil, 'count_until')

    def send_goal(self, target_number, delay):
        self.action_client_.wait_for_server()

        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.delay = delay

        self.get_logger().info("Sending goal...")
        self.action_client_.send_goal_async(
            goal,
            feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_ = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted!")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback)
        else:
            self.get_logger().warn("Goal rejected.")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Success! Reached: {result.reached_number}")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal was aborted.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal was canceled.")

    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.current_number}")

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(target_number=5, delay=0.5)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()