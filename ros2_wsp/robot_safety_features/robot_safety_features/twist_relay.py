#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")

        # Subscribers
        self.keyboard_sub = self.create_subscription(
            Twist, "/keyboard_input/cmd_vel", self.twist_callback, 10)
        self.controller_sub = self.create_subscription(
            Twist, "/motor_control/cmd_vel_unstamped", self.twist_callback, 10)
        self.safety_stop_sub = self.create_subscription(
            Bool, "/safety_stop", self.safety_stop_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # State variables
        self.safety_stop_active = False
        self.zero_twist = Twist()

    def twist_callback(self, msg):
        """Publishes Twist messages if safety stop is not active."""
        if not self.safety_stop_active:
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info("Forwarding command to /cmd_vel.")
        else:
            self.cmd_vel_pub.publish(self.zero_twist)
            self.get_logger().info("Safety stop active: Publishing zero velocity.")

    def safety_stop_callback(self, msg):
        """Updates the safety stop state."""
        self.safety_stop_active = msg.data
        if self.safety_stop_active:
            self.cmd_vel_pub.publish(self.zero_twist)
            self.get_logger().info("Safety stop engaged: Robot stopped.")
        else:
            self.get_logger().info("Safety stop disengaged: Commands allowed.")


def main():
    rclpy.init()
    node = TwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
