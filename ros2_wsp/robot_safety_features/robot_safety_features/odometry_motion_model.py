#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import euler_from_quaternion
from math import sqrt, atan2, pow, pi

def angle_diff(a, b):
    diff = a - b
    while diff > pi:
        diff -= 2 * pi
    while diff < -pi:
        diff += 2 * pi
    return diff

class OdometryMotionModel(Node):
    def __init__(self):
        super().__init__("odometry_motion_model")
        
        # Initialize variables
        self.is_first_odom = True
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        
        # Declare parameters
        self.declare_parameter("alpha1", 0.1)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        self.declare_parameter("nr_samples", 300)

        # Get parameter values
        self.alpha1 = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2 = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3 = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4 = self.get_parameter("alpha4").get_parameter_value().double_value
        self.nr_samples = self.get_parameter("nr_samples").get_parameter_value().integer_value

        # Validate number of samples
        if self.nr_samples < 0:
            self.get_logger().fatal("Invalid number of samples requested: %d", self.nr_samples)
            return
        else:
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.nr_samples)]

        # Initialize subscriptions and publishers
        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.odomCallback, 10)
        self.pose_array_pub_ = self.create_publisher(PoseArray, "odometry_motion_model/samples", 10)

    def odomCallback(self, odom):
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw
            self.samples.header.frame_id = odom.header.frame_id
            self.is_first_odom = False
            return

        # Calculate odometry increments
        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)

        if sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2)) < 0.01:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), self.last_odom_theta)

        delta_trasl = sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2))
        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)

        # Here you would update the pose samples and publish if needed
        self.get_logger().info(f"Delta Rot1: {delta_rot1}, Delta Trasl: {delta_trasl}, Delta Rot2: {delta_rot2}")
        
        # Update the last odometry values
        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

def main(args=None):
    # Initialize the rclpy library and create a node instance
    rclpy.init(args=args)
    node = OdometryMotionModel()

    # Spin the node to keep it running
    rclpy.spin(node)

    # Shut down the node after spinning
    rclpy.shutdown()

if __name__ == "__main__":
    main()
