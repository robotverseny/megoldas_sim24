# original from: https://github.com/SZE-F1TENTH/followthegap/blob/main/follow_the_gap/follow_the_gap_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        
        # Parameters
        self.safety_radius = 2   # Minimum safe distance from obstacles
        self.max_throttle = 0.5   # Maximum throttle value
        self.steering_sensitivity = -0.9  # Adjust sensitivity as needed
        self.max_steering_angle = 0.52   # Steering angle limit in radians

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, '/debug_marker', 1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pubst1 = self.create_publisher(String, 'control_state', 10)
        self.get_logger().info('Follow the gap node has been started', once=True)


    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)

        # Replace 'inf' values with max range
        max_range = scan_data.range_max
        ranges[np.isinf(ranges)] = max_range

        # Only keep ranges beyond the safety radius
        safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)

        # Find the largest safe gap and its midpoint angle
        best_direction = self.find_best_gap(safe_ranges, scan_data.angle_min, scan_data.angle_increment)

        # Publish throttle and steering commands
        self.publish_drive_command(best_direction)

    def find_best_gap(self, ranges, angle_min, angle_increment):
        # Identify indices of safe ranges
        safe_indices = np.where(ranges > 0)[0]

        # Find gaps and select the largest
        gaps = []
        gap_start = safe_indices[0]
        for i in range(1, len(safe_indices)):
            if safe_indices[i] - safe_indices[i - 1] > 1:
                gaps.append((gap_start, safe_indices[i - 1]))
                gap_start = safe_indices[i]
        gaps.append((gap_start, safe_indices[-1]))

        # Select the largest gap by its width
        largest_gap = max(gaps, key=lambda gap: gap[1] - gap[0])

        # Calculate the middle angle of the largest gap
        mid_index = (largest_gap[0] + largest_gap[1]) // 2
        best_angle = angle_min + mid_index * angle_increment
        # Create a marker for the center of the gap
        marker_center = Marker()
        marker_center.header.frame_id = "laser"
        marker_center.ns = "follow_the_gap"
        marker_center.id = 0
        marker_center.type = Marker.SPHERE
        marker_center.action = Marker.ADD
        marker_center.pose.position.x = ranges[mid_index] * np.cos(best_angle)
        marker_center.pose.position.y = ranges[mid_index] * np.sin(best_angle)
        marker_center.pose.position.z = 0.0
        marker_center.pose.orientation.x = 0.0; marker_center.pose.orientation.y = 0.0; marker_center.pose.orientation.z = 0.0; marker_center.pose.orientation.w = 1.0
        marker_center.scale.x = 0.6; marker_center.scale.y = 0.6; marker_center.scale.z = 0.6
        marker_center.color.a = 1.0
        marker_center.color.r = 0.2
        marker_center.color.g = 0.6
        marker_center.color.b = 0.4
        marker_array = MarkerArray()
        # Add to the marker array and publish
        marker_array.markers.append(marker_center)
        self.debug_marker_pub.publish(marker_array)
        # Publish control state as a string message
        messageS1 = String()
        messageS1.data = "Follow_the_gap"
        messageS1.data += f"\nmid_index: {mid_index:.0f}"
        # messageS1.data += f"\nbest_angle: {best_angle:.2f}"
        messageS1.data += f"\nbest_angle (deg): {np.degrees(best_angle):.1f}"
        messageS1.data += f"\nsteering_sentivity: {self.steering_sensitivity:.1f}"
        self.pubst1.publish(messageS1)
        return best_angle

    def publish_drive_command(self, best_angle):
        # Throttle command (constant for simplicity)
        throttle_value = self.max_throttle
        # self.throttle_pub.publish(throttle_msg)
        
        # Steering command, [-0.52, 0.52] range
        steering_value = -best_angle * self.steering_sensitivity
        # self.steering_pub.publish(steering_msg)
        twist_cmd = Twist()
        twist_cmd.linear.x = throttle_value
        twist_cmd.angular.z = steering_value
        self.cmd_pub.publish(twist_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()