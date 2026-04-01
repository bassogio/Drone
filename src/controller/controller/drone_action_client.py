#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from drone_interfaces.action import DroneCommand

class DroneActionClient(Node):
    def __init__(self):
        super().__init__('drone_action_client')
        
        # ===============
        #   Client
        # ===============
        self._action_client = ActionClient(
            self, 
            DroneCommand,
            'drone_command')

    def send_goal(self, command, target_x, target_y, target_z, target_yaw):
        self._action_client.wait_for_server()
        
        goal_msg = DroneCommand.Goal()
        goal_msg.command = command
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.target_z = target_z
        goal_msg.target_yaw = target_yaw
        
        self.get_logger().info(f"Sending {command} goal")
        
        # Send the goal - Without feedback
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        
        # Send the goal - With feedback
        # self._action_client.send_goal_async(
        #     goal_msg,
        #     feedback_callback=self.goal_feedback_callback
        #     ).add_done_callback(self.goal_response_callback)
    
    def goal_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"state={fb.state} | "
            f"x={fb.current_x:.2f}, y={fb.current_y:.2f}, z={fb.current_z:.2f}, "
            f"yaw={fb.current_yaw:.2f}, dist={fb.distance_to_goal:.2f}")
    
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().error("Goal got rejected")
            self.destroy_node()
            rclpy.shutdown()
            
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Cancled")
        self.get_logger().info("Result: " + str(result.message))
        self.destroy_node()
        rclpy.shutdown()
        
def error_input_msg(node, reason=None):
    msg = "\nInvalid action client input."

    if reason:
        msg += f"\nReason: {reason}"

    msg += (
        "\n\nHow to use:"
        "\n  ros2 run controller action_client <COMMAND> [arguments]"
        "\n"
        "\nCommands:"
        "\n  TAKEOFF [z]"
        "\n      Take off to the requested z position."
        "\n      z is optional."
        "\n      Example: TAKEOFF 3"
        "\n"
        "\n  HOVER"
        "\n      Hold the current position."
        "\n      Extra arguments are not allowed."
        "\n"
        "\n  MOVE [x] [y] [z] [yaw]"
        "\n      Move to a target position and yaw."
        "\n      You can provide 0 to 4 numeric arguments."
        "\n      Missing values default to 0.0."
        "\n      Example: MOVE 1 2 3 0"
        "\n"
        "\n  LAND"
        "\n      Land the drone."
        "\n      Extra arguments are not allowed."
        "\n"
        "\nExamples:"
        "\n  ros2 run controller action_client TAKEOFF 3"
        "\n  ros2 run controller action_client HOVER"
        "\n  ros2 run controller action_client LAND"
        "\n  ros2 run controller action_client MOVE 1 2 3 0"
        "\n"
        "\nNotes:"
        "\n  - Command names are case-insensitive."
        "\n  - All numeric values must be valid numbers."
        "\n  - If your controller uses NED, negative z means up."
    )

    node.get_logger().error(msg)

def parse_command(argv):
    if len(argv) < 2:
        raise ValueError("Missing command")

    command = argv[1].upper()

    if command in {"HOVER", "LAND"}:
        if len(argv) != 2:
            raise ValueError(f"{command} does not take extra arguments")
        return command, float('nan'), float('nan'), float('nan'), float('nan')

    if command == "TAKEOFF":
        if len(argv) not in {2, 3}:
            raise ValueError("TAKEOFF accepts zero or one argument")
        target_z = -float(argv[2]) if len(argv) == 3 else float('nan')
        return command, float('nan'), float('nan'), target_z, float('nan')

    if command == "MOVE":
        if len(argv) > 6 or len(argv) < 3:
            raise ValueError("MOVE accepts minimun 1 and up to 4 numeric arguments")

        values = [float(x) for x in argv[2:]]
        
        while len(values) < 4:
            values.append(float('nan'))

        target_x, target_y, target_z, target_yaw = values
                
        return command, target_x, target_y, -target_z, target_yaw

    raise ValueError(f"Unknown command: {command}")

def main(args=None):
    rclpy.init(args=args)

    drone_action_client = DroneActionClient()
    
    try:
        command, target_x, target_y, target_z, target_yaw = parse_command(sys.argv)
    except ValueError as e:
        drone_action_client.get_logger().error(str(e))
        error_input_msg(drone_action_client)
        drone_action_client.destroy_node()
        rclpy.shutdown()
        return

    drone_action_client.send_goal(command, target_x, target_y, target_z, target_yaw)

    rclpy.spin(drone_action_client)


if __name__ == '__main__':
    main()