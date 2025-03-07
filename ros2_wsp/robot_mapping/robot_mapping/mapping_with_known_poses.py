#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException


# Constants for probabilities
PROIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

class Pose:
    def __init__(self, px=0, py=0):
        self.x = px
        self.y = py

def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return 0 <= pose.x < map_info.width and 0 <= pose.y < map_info.height

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

def bresenham(start: Pose, end: Pose):
    line = []
    dx = abs(end.x - start.x)
    dy = abs(end.y - start.y)
    xsign = 1 if end.x > start.x else -1
    ysign = 1 if end.y > start.y else -1

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2 * dy - dx
    y = 0

    for x in range(dx + 1):
        line.append(Pose(start.x + x * xx + y * yx, start.y + x * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inverseSensorModel(p_robot: Pose, p_beam: Pose):
    occ_values = []
    line = bresenham(p_robot, p_beam)
    
    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))
    occ_values.append((line[-1], OCC_PROB))
    return occ_values

def prob2logodds(p):
    return math.log(p / (1-p))

def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

        # Declare parameters
        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        # Get parameters
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        # Initialize map
        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "map1"  # Use "map1" as the custom mapping frame
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.probability_map_ = [prob2logodds(PROIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        # Publishers and subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, "map1", 1)  # Publish the custom map
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_callback(self, scan: LaserScan):
        try:
            # Use "map1" and "base_footprint" to align with your setup
            t = self.tf_buffer.lookup_transform(
                "map1", "base_footprint", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except (LookupException, TimeoutException):
            self.get_logger().error("Unable to transform between /map1 and /base_footprint")
            return

        # Convert robot's position to map coordinates
        robot_pose = coordinatesToPose(
            t.transform.translation.x, t.transform.translation.y, self.map_.info
        )

        if not poseOnMap(robot_pose, self.map_.info):
            self.get_logger().error("The robot is out of the map!")
            return

        (roll, pitch, yaw) = euler_from_quaternion([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ])

        for i, range_val in enumerate(scan.ranges):
            if math.isinf(range_val):
                continue

            angle = scan.angle_min + (i * scan.angle_increment) + yaw
            px = range_val * math.cos(angle) + t.transform.translation.x
            py = range_val * math.sin(angle) + t.transform.translation.y

            beam_p = coordinatesToPose(px, py, self.map_.info)

            if not poseOnMap(beam_p, self.map_.info):
                continue

            poses = inverseSensorModel(robot_pose, beam_p)
            for pose, value in poses:
                cell = poseToCell(pose, self.map_.info)
                if 0 <= cell < len(self.probability_map_):  # Ensure cell index is valid
                    self.probability_map_[cell] += prob2logodds(value) - prob2logodds(PROIOR_PROB)

    def timer_callback(self):
        # Publish the map periodically
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_.data = [int(logodds2prob(value) * 100) for value in self.probability_map_]
        self.map_pub.publish(self.map_)

def main():
    rclpy.init()
    node = MappingWithKnownPoses("mapping_with_known_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
