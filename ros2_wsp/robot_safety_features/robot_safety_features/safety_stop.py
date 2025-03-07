#!/usr/bin/env python3
import math
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray


class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2


class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        # Declare and get parameters
        self.declare_parameter("danger_distance", 0.5)  # Stop if an object is closer than 0.5m
        self.declare_parameter("warning_distance", 1.0)  # Warn if an object is closer than 1.0m
        self.declare_parameter("robot_clearance_distance", 0.1)  # Ignore objects closer than 0.2m
        self.declare_parameter("angle_range", 10.0)  # Monitor ±10 degrees
        self.declare_parameter("scan_topic", "scan")  # Input LaserScan topic
        self.declare_parameter("safety_stop_topic", "safety_stop")  # Output safety stop topic
        self.declare_parameter("object_distance_topic", "object_distance")  # Detected object distance topic

        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.robot_clearance_distance = self.get_parameter("robot_clearance_distance").get_parameter_value().double_value
        self.angle_range = math.radians(self.get_parameter("angle_range").get_parameter_value().double_value)
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value
        self.object_distance_topic = self.get_parameter("object_distance_topic").get_parameter_value().string_value

        # Subscribers and Publishers
        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.object_distance_pub = self.create_publisher(Float32, self.object_distance_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, "zones", 10)
        self.object_marker_pub = self.create_publisher(Marker, "object_marker", 10)

        # State and visualization markers
        self.state = State.FREE
        self.zones = self.create_markers()

    def create_markers(self):
        """Create visualization markers for warning and danger zones."""
        markers = MarkerArray()

        warning_zone = Marker()
        warning_zone.id = 0
        warning_zone.action = Marker.ADD
        warning_zone.type = Marker.CYLINDER
        warning_zone.scale.z = 0.001  # Flat cylinder
        warning_zone.scale.x = self.warning_distance * 2
        warning_zone.scale.y = self.warning_distance * 2
        warning_zone.color.r = 1.0
        warning_zone.color.g = 1.0
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.5
        warning_zone.pose.position.z = 0.01

        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.action = Marker.ADD
        danger_zone.type = Marker.CYLINDER
        danger_zone.scale.z = 0.001  # Flat cylinder
        danger_zone.scale.x = self.danger_distance * 2
        danger_zone.scale.y = self.danger_distance * 2
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.5
        danger_zone.pose.position.z = 0.01

        markers.markers = [warning_zone, danger_zone]
        return markers

    def laser_callback(self, msg: LaserScan):
        """Callback function to process LIDAR scan data and publish safety stop signals."""
        # Calculate indices for the angle range
        center_index = int((0.0 - msg.angle_min) / msg.angle_increment)
        angle_range_indices = int(self.angle_range / msg.angle_increment)
        start_index = max(0, center_index - angle_range_indices)
        end_index = min(len(msg.ranges), center_index + angle_range_indices)

        self.state = State.FREE
        min_distance = float("inf")  # Initialize minimum distance
        min_distance_angle = 0.0

        for i in range(start_index, end_index):
            range_value = msg.ranges[i]

            # Ignore invalid, infinite, or self-detection ranges
            if not math.isfinite(range_value) or range_value < self.robot_clearance_distance:
                continue

            # Track the minimum distance and angle
            if range_value < min_distance:
                min_distance = range_value
                min_distance_angle = msg.angle_min + i * msg.angle_increment

            # Check for danger state
            if range_value <= self.danger_distance:
                self.state = State.DANGER
                break
            elif range_value <= self.warning_distance:
                self.state = State.WARNING

        # Publish the safety stop signal
        is_safety_stop = Bool()
        is_safety_stop.data = self.state == State.DANGER
        self.safety_stop_pub.publish(is_safety_stop)

        # Publish the object distance to the topic
        object_distance_msg = Float32()
        object_distance_msg.data = min_distance if min_distance != float("inf") else -1.0  # -1.0 means no object detected
        self.object_distance_pub.publish(object_distance_msg)

        # Publish a marker for the detected object
        if min_distance != float("inf"):
            object_marker = Marker()
            object_marker.header.frame_id = msg.header.frame_id
            object_marker.header.stamp = self.get_clock().now().to_msg()
            object_marker.id = 2
            object_marker.type = Marker.SPHERE
            object_marker.action = Marker.ADD
            object_marker.pose.position.x = min_distance * math.cos(min_distance_angle)
            object_marker.pose.position.y = min_distance * math.sin(min_distance_angle)
            object_marker.pose.position.z = 0.0
            object_marker.scale.x = 0.1
            object_marker.scale.y = 0.1
            object_marker.scale.z = 0.1
            object_marker.color.r = 0.0
            object_marker.color.g = 1.0
            object_marker.color.b = 0.0
            object_marker.color.a = 1.0
            self.object_marker_pub.publish(object_marker)

        # Update markers for visualization
        self.update_markers(msg.header.frame_id)

    def update_markers(self, frame_id):
        """Update visualization markers based on the current state."""
        for zone in self.zones.markers:
            zone.header.frame_id = frame_id

        if self.state == State.WARNING:
            self.zones.markers[0].color.a = 1.0  # Warning zone active
            self.zones.markers[1].color.a = 0.5
        elif self.state == State.DANGER:
            self.zones.markers[0].color.a = 1.0
            self.zones.markers[1].color.a = 1.0  # Danger zone active
        else:
            self.zones.markers[0].color.a = 0.5
            self.zones.markers[1].color.a = 0.5

        self.zones_pub.publish(self.zones)


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
