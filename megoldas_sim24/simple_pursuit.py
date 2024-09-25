import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped, Transform
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs

KOZEPISKOLA_NEVE = "Ismeretlen kozepiskola"
KOZEPISKOLA_AZON = "A00"
ANGLE_RANGE = 360  # LSN10 LIDAR has 360 degrees scan
DESIRED_DISTANCE_RIGHT = 1.0  # meters
DESIRED_DISTANCE_LEFT = 0.8  # meters
VELOCITY = 1.00  # meters per second
CAR_LENGTH = 0.445  # meters
WHEELBASE = 0.3187  # meters

class SimplePursuit(Node):
    def __init__(self):
        super().__init__('simple_pursuit')
        self.init_publishers()
        self.init_subscribers()
        self.init_markers()
        self.init_tf2()
        self.first_run = True
        self.prev_steering_err = 0.0
        self.prev_velocity = 0.0
        self.trans = Transform()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def init_publishers(self):
        self.pub = self.create_publisher(Twist, 'roboworks/cmd_vel', 1)
        self.pubst1 = self.create_publisher(String, 'control_state', 10)
        self.pubst2 = self.create_publisher(String, 'kozepiskola', 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, '/debug_marker', 1)

    def init_subscribers(self):
        self.sub = self.create_subscription(LaserScan, 'roboworks/scan', self.callbackLaser, 1)

    def init_markers(self):
        self.marker_points_left = self.create_marker(0.0, 0.2, 0.8, "left")
        self.marker_points_right = self.create_marker(1.0, 0.0, 0.0, "right")
        self.marker_points_goal = self.create_marker(0.6, 0.2, 0.5, "goal")
        self.debugMarkerArray = MarkerArray()

    def create_marker(self, r, g, b, ns):
        marker = Marker()
        marker.header.frame_id = "map_roboworks"
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.MODIFY
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.pose.orientation.w = 1.0
        marker.ns = ns
        return marker

    def init_tf2(self):
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def calcPursuitAngle(self, goal_x, goal_y):
        alpha = math.atan2(goal_y, goal_x)
        lookahead_distance = math.sqrt(goal_x**2 + goal_y**2)
        steering_angle = math.atan2(2.0 * WHEELBASE * np.sin(alpha) / lookahead_distance, 1)
        return steering_angle

    def calcPointPos(self, range, angle):
        x = range * math.cos(angle)
        y = range * math.sin(angle)
        return x, y

    def getDistance(self, ranges, angles):
        if len(ranges) <= 50:
            return 0.4

        max_x = -10.0
        lookup_right = self.get_lookup_indices(angles, -20, -0.1)
        lookup_left = self.get_lookup_indices(angles, 0.1, 20)
        lookup = np.concatenate((lookup_right, lookup_left))

        for t in lookup:
            point_x, _ = self.calcPointPos(ranges[t], angles[t])
            if not math.isinf(point_x) and point_x > max_x:
                max_x = point_x

        if math.isinf(max_x):
            max_x = -5.0
        if max_x < 0.7:
            max_x = -0.5

        return max_x

    def get_lookup_indices(self, angles, min_angle, max_angle):
        min_index = np.where(math.radians(min_angle) < angles)[0][0]
        max_index = np.where(math.radians(max_angle) < angles)[0][0]
        return np.arange(min_index, max_index, 1)

    def getAngle(self, ranges, angles):
        if len(ranges) <= 50:
            return 0.0, -99.0, 99.0

        left_d = self.get_side_distance(ranges, angles, 30, 60, self.marker_points_left)
        right_d = self.get_side_distance(ranges, angles, -60, -30, self.marker_points_right)
        angle = (left_d + right_d) / 2
        if math.isinf(right_d):
            right_d = 99.0
            angle = 0.0
        if math.isinf(left_d):
            left_d = -99.0
            angle = 0.0

        return angle, left_d, right_d

    def get_side_distance(self, ranges, angles, min_angle, max_angle, marker, invert=False):
        min_index = np.where(math.radians(min_angle) < angles)[0][0]
        max_index = np.where(math.radians(max_angle) < angles)[0][0]
        indices = np.arange(min_index, max_index, 1)

        side_d = -10.0 if not invert else 10.0
        for t in indices:
            point_x, point_y = self.calcPointPos(ranges[t], angles[t])
            if not math.isinf(point_y):
                if not invert and point_y > side_d:
                    side_d = point_y
                elif invert and point_y < side_d:
                    side_d = -point_y
            marker.points.append(Point(x=point_x, y=point_y))

        return side_d

    def followSimple(self, data):
        messageS1 = String()
        messageS1.data = "Simple_pursuit"
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        if len(angles) != len(data.ranges):
            self.get_logger().warn("angles and ranges length differ")

        target_distance = self.getDistance(data.ranges, angles)
        target_angle, left_d, right_d = self.getAngle(data.ranges, angles)

        point = Point(x=target_distance, y=target_angle)
        point_st = PointStamped(point=point)
        point_base_link_frame = None
        
        try:
            point_base_link_frame = tf2_geometry_msgs.do_transform_point(point_st, self.trans)
            self.marker_points_goal.points.append(point_base_link_frame.point)
            
        except:
            self.get_logger().error("Error in transforming point")

        self.publish_markers()
        if point_base_link_frame:
            self.update_message(messageS1, target_angle, left_d, right_d, target_distance, point_base_link_frame)
        self.pubst1.publish(messageS1)

        if point_base_link_frame:
            steering_err, velocity = self.calculate_control(point_base_link_frame)
        else:
            steering_err, velocity = 0.0, 0.0  
        return steering_err, velocity

    def publish_markers(self):
        self.debugMarkerArray.markers = [self.marker_points_left, self.marker_points_right, self.marker_points_goal]
        self.debug_marker_pub.publish(self.debugMarkerArray)
        self.marker_points_left.points = []
        self.marker_points_right.points = []
        self.marker_points_goal.points = []

    def update_message(self, message, target_angle, left_d, right_d, target_distance, point_base_link_frame):
        message.data += f"\ntarget_angle: {target_angle:.1f}"
        message.data += f"\nr: {right_d:.1f} l: {left_d:.1f}"
        message.data += f"\ntarget_distance: {target_distance:.1f}"
        try:
            steering_err = self.calcPursuitAngle(point_base_link_frame.point.x, point_base_link_frame.point.y)
        except:
            steering_err = self.calcPursuitAngle(1, -1)
        message.data += f"\nsteer: {steering_err:.1f}"
        message.data += f"\nvelocity: {-1.0 * target_distance:.1f}"

    def calculate_control(self, point_base_link_frame):
        try:
            steering_err = self.calcPursuitAngle(point_base_link_frame.point.x, point_base_link_frame.point.y)
        except:
            steering_err = self.calcPursuitAngle(1, -1)

        steering_err = (steering_err + self.prev_steering_err) / 2
        velocity = (point_base_link_frame.point.x + self.prev_velocity) / 2
        self.prev_steering_err = steering_err
        self.prev_velocity = velocity
        return steering_err, velocity

    def callbackLaser(self, data):
        error_steering, velocity = self.followSimple(data)
        msg_cmd = Twist()
        msg_cmd.linear.x = velocity * -0.5
        msg_cmd.angular.z = error_steering
        self.pub.publish(msg_cmd)

    def timer_callback(self):
        if self.first_run:
            try:
                self.trans = self.buf.lookup_transform('map_roboworks', 'lidar_link', rclpy.time.Time())
                self.first_run = False
            except tf2_ros.TransformException as ex:
                self.get_logger().info(f'Could not transform {"map_roboworks"} to {"lidar_link"}: {ex}')
                self.set_default_transform()
        self.pubst2.publish(String(data=f"{KOZEPISKOLA_NEVE} ({KOZEPISKOLA_AZON})"))

    def set_default_transform(self):
        self.trans.translation.x = 0.26
        self.trans.translation.y = 0.0
        self.trans.translation.z = 0.228
        self.trans.rotation.x = 0.0
        self.trans.rotation.y = 0.0
        self.trans.rotation.z = 0.999999682932
        self.trans.rotation.w = 0.000796326710733

def main(args=None):
    rclpy.init(args=args)
    node = SimplePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()