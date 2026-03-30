#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
# Replace with your action interface:
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__('count_until_server')

        # ActionServer(node, ActionType, action_name, goal_cb, execute_cb)
        self.action_server_ = ActionServer(
            self,
            CountUntil,
            'count_until',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback)
        self.get_logger().info("Action Server started.")

    def goal_callback(self, goal_request: CountUntil.Goal):
        """Accept or reject an incoming goal."""
        self.get_logger().info("Received a goal")
        if goal_request.target_number <= 0:
            self.get_logger().warn("Rejecting: target_number must be positive")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the accepted goal."""
        target_number = goal_handle.request.target_number
        delay = goal_handle.request.delay
        result = CountUntil.Result()
        counter = 0

        self.get_logger().info("Executing goal...")
        for i in range(target_number):
            counter += 1
            self.get_logger().info(f"Count: {counter}")
            time.sleep(delay)

        # Mark goal as succeeded and return result
        goal_handle.succeed()
        result.reached_number = counter
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()