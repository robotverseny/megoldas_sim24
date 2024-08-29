#!/usr/bin/env python3
'''
based on 2018 Varundev Suresh Babu (University of Virginia)
                MIT License
'''

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped, Transform
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs

KOZEPISKOLA_NEVE = "Ismeretlen kozepiskola"
KOZEPISKOLA_AZON = "A00"
ANGLE_RANGE = 360 # LSN10 LIDAR has 360 degrees scan
DESIRED_DISTANCE_RIGHT = 1.0 #0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8 # 0.55
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.445 # 0.445 meters
WHEELBASE = 0.3187; # documention based | measured: ~32 cm

class SimplePursuit(Node):
    def __init__(self):
        super().__init__('simple_pursuit')
        self.pub = self.create_publisher(Twist, 'roboworks/cmd_vel', 1)
        self.pubst1 = self.create_publisher(String, 'pid_data', 10)
        self.pubst2 = self.create_publisher(String, 'kozepiskola', 10)
        self.marker_pub = self.create_publisher(Marker, '/debug_marker', 1)
        self.sub = self.create_subscription(LaserScan, 'roboworks/scan', self.callbackLaser, 10)
        self.prev_steering_err = 0.0
        self.prev_velocity = 0.0
        self.trans = Transform()
        self.marker_points = Marker()
        self.marker_points.header.frame_id = "roboworks/lidar_link"
        self.marker_points.type = Marker.SPHERE_LIST
        self.marker_points.action = Marker.MODIFY
        self.marker_points.color.r = 0.0
        self.marker_points.color.g = 0.0
        self.marker_points.color.a = 1.0
        self.marker_points.color.b = 1.0
        self.marker_points.scale.x = 0.4
        self.marker_points.scale.y = 0.4
        self.marker_points.scale.z = 0.4
        self.marker_points.pose.orientation.x = 0.0
        self.marker_points.pose.orientation.y = 0.0
        self.marker_points.pose.orientation.z = 0.0
        self.marker_points.pose.orientation.w = 1.0
        self.first_run = True
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def calcPursuitAngle(self, goal_x, goal_y):
        alpha = math.atan2(goal_y, goal_x)
        lookahead_distance = math.sqrt(pow(goal_x, 2) + pow(goal_y, 2))
        steering_angle = math.atan2(2.0 * WHEELBASE * np.sin(alpha) / (lookahead_distance), 1)
        return steering_angle

    def calcPointPos(self, range, angle):
        x1 = range * math.cos(angle)
        y1 = range * math.sin(angle)
        return x1, y1


    def getDistance(self, ranges, angles):
        if(len(ranges) > 50):
            center1_min_index = np.where(math.radians(-20) < angles)[0][0]
            center1_max_index = np.where(math.radians(0) > angles)[0][0]
            tmp1 = np.arange(center1_min_index, center1_max_index, 1)
            center2_min_index = np.where(math.radians(0) < angles)[0][0]
            center2_max_index = np.where(math.radians(20) < angles)[0][0]
            tmp2 = np.arange(center2_min_index, center2_max_index, 1)
            tmp=np.concatenate((tmp1, tmp2))
            max_x = 10.0
            # lookup_data = np.arange(0, len(ranges), 1) 
            # lookup_right_min= np.where(math.radians(0) < angles)[0][0]
            # lookup_right_max= np.where(math.radians(-20)  < angles)[0][0]
            # lookup_right= np.arange(lookup_right_min, lookup_right_max,1)
            # lookup_left_min= np.where(math.radians(0) < angles)[0][0]
            # lookup_left_max= np.where(math.radians(20)  < angles)[0][0]
            # lookup_left= np.arange(lookup_left_min, lookup_left_max,1)
            # lookup = np.concatenate(lookup_right, lookup_left)
            for t in tmp:
                point = Point()
                point.x, point.y = self.calcPointPos(ranges[t], angles[t])
                if not math.isinf(point.x):
                    if point.x > max_x:
                        max_x = point.x
                
            if math.isinf(max_x):
                max_x = 5.0
            if max_x > 0.4:
                max_x = -0.5
            distance = max_x
        else: 
            distance = 0.4
        return distance

    def getAngle(self, ranges, angles):
        if len(ranges) > 50:
            left1_min_index = np.where(math.radians(-60) < angles)[0][0]
            left1_max_index = np.where(math.radians(-30) < angles)[0][0]
            tmp_left = np.arange(left1_min_index, left1_max_index, 1)
            
            left_d = 10.0
            for t in tmp_left:
                point = Point()
                point.x, point.y = self.calcPointPos(ranges[t], angles[t])
                if not math.isinf(point.y):
                    if left_d < point.y:
                        left_d = point.y
                self.marker_points.points.append(point)
            
            right1_min_index = np.where(math.radians(30) < angles)[0][0]
            right1_max_index = np.where(math.radians(60) < angles)[0][0]
            tmp_right = np.arange(right1_min_index, right1_max_index, 1)
            
            right_d = -10.0
            for t in tmp_right:
                point = Point()
                point.x, point.y = self.calcPointPos(ranges[t], angles[t])
                if not math.isinf(point.y):
                    if point.y < right_d:
                        right_d = point.y
                self.marker_points.points.append(point)
            angle = (left_d + right_d) / 2
            if math.isinf(right_d):
                right_d = 99.0
                angle = 0.0
            if math.isinf(left_d):
                left_d = -99.0
                angle = 0.0
        else:
            angle = 0.0
            left_d = -99.0
            right_d = 99.0

        print(f"Angle: {angle}, Left Distance: {left_d}, Right Distance: {right_d}")
        return angle, left_d, right_d

    def followSimple(self, data):
        messageS1 = String()
        messageS1.data = "Egyszeru_pursuit"
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        if (len(angles) - len(data.ranges) != 0):
            self.get_logger().warn("angles and ranges length differ")

        target_distance = self.getDistance(data.ranges, angles)
        target_angle, left_d, right_d = self.getAngle(data.ranges, angles)
        

        point = Point()
        point.x = target_distance
        point.y = target_angle
        point_st = PointStamped()
        point_st.point = point

        try:
            point_base_link_frame = tf2_geometry_msgs.do_transform_point(point_st, self.trans)
            point_base_link_frame.point.x *= -0.9
            self.marker_points.points.append(point_base_link_frame.point)
        except:
            pass

        self.marker_pub.publish(self.marker_points)

        messageS1.data += "\ntarget_angle: %.1f" % (target_angle)
        messageS1.data += "\nr: %.1f l: %.1f" % (right_d, left_d) 
        messageS1.data += "\nforward_d: %.1f" % (target_distance)
        velocity = -1.0 * target_distance
        try:
            steering_err = self.calcPursuitAngle(point_base_link_frame.point.x, point_base_link_frame.point.y)
        except:
            print("Error in calcPursuitAngle")
            steering_err = self.calcPursuitAngle(1, -1)
            
        messageS1.data += "\nsteer: %.1f" % (steering_err)
        messageS1.data += "\nvelocity: %.1f" % (velocity)
        self.pubst1.publish(messageS1)
        self.marker_points.points = []
        steering_err = (steering_err + self.prev_steering_err) / 2
        velocity = (velocity + self.prev_velocity) / 2
        self.prev_steering_err = steering_err
        self.prev_velocity = velocity
        return steering_err, velocity

    def callbackLaser(self, data):
        error_steering, velocity = self.followSimple(data)
        msg_cmd = Twist()
        msg_cmd.linear.x = velocity * 0.5
        msg_cmd.angular.z = error_steering
        self.pub.publish(msg_cmd)

    def timer_callback(self):
        if self.first_run:
            try:
                self.trans = self.buf.lookup_transform("roboworks/odom", "roboworks/lidar_link", rclpy.time.Time())
                self.get_logger().info("Got laser transform")
                self.first_run = False
            except:
                self.trans.translation.x = 0.26
                self.trans.translation.y = 0.0
                self.trans.translation.z = 0.228
                self.trans.rotation.x = 0.0
                self.trans.rotation.y = 0.0
                self.trans.rotation.z = 0.999999682932
                self.trans.rotation.w = 0.000796326710733
        self.pubst2.publish(String(data=KOZEPISKOLA_NEVE + "(" + KOZEPISKOLA_AZON + ")"))

def main(args=None):
    rclpy.init(args=args)
    node = SimplePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()