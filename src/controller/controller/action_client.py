#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from drone_interfaces.action import DroneCommand

class DroneActionClient(Node):
    def __init__(self):
        super().__init__("drone_action_client")
        self._client = ActionClient(self, DroneCommand, "drone_command")

    def send_goal(self, command, x=0.0, y=0.0, z=0.0, yaw=0.0):
        goal_msg = DroneCommand.Goal()
        goal_msg.command = command
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.yaw = float(yaw)

        self._client.wait_for_server()

        self.get_logger().info(f"Sending goal: {command}")
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"state={fb.state} | "
            f"x={fb.current_x:.2f}, y={fb.current_y:.2f}, z={fb.current_z:.2f}, "
            f"yaw={fb.current_yaw:.2f}, dist={fb.distance_to_goal:.2f}"
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result: success={result.success}, message='{result.message}'"
        )
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DroneActionClient()

    if len(sys.argv) < 2:
        node.get_logger().error(
            "Usage:\n"
            "  ros2 run controller action_client TAKEOFF 0 0 -3 0\n"
            "  ros2 run controller action_client MOVE 1 2 -3 0\n"
            "  ros2 run controller action_client LAND"
        )
        return

    command = sys.argv[1].upper()

    x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0

    node.send_goal(command, x, y, z, yaw)
    rclpy.spin(node)


if __name__ == "__main__":
    main()