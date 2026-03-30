import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class StatePublisherNode(Node):
    def __init__(self):
        super().__init__("state_publisher")

        self.x = None
        self.y = None
        self.z = None
        self.yaw = None
        self.arming_state = None
        self.nav_state = None

        self.position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(1.0, self.log_state)

    def log_state(self):
        # Start printing only when all data is received
        if None in [self.x, self.y, self.z, self.yaw]:
            return

        self.get_logger().info(f"x={self.x:.3f} | y={self.y:.3f} | z={self.z:.3f} | yaw={self.yaw:.3f}")
        
    def position_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.yaw = msg.heading

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()