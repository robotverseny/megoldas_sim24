import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from control_msgs.msg import PidState
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.pub = self.create_publisher(Twist, 'roboworks/cmd_vel', 1)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 1)

        self.kp = 9.5
        self.kd = 0.0
        self.kp_vel = 42.0
        self.kd_vel = 0.0
        self.ki = 0.0
        self.servo_offset = 18.0 * math.pi / 180
        self.prev_error = 0.0
        self.error = 0.0
        self.vel_input = 1.0
        self.marker_id = 0

        self.subscription = self.create_subscription(
            PidState,
            'error',
            self.control,
            1
        )

    def control(self, data):
        self.error = 5 * data.p_error

        if self.error != 0.0:
            control_error = self.kp * self.error + self.kd * (self.error - self.prev_error)
            angle = self.servo_offset + control_error * np.pi / 180
            control_error_vel = self.kp_vel * self.error + self.kd_vel * (self.error - self.prev_error)
            velocity = data.p_term + abs(control_error_vel)

            self.prev_error = self.error

            if angle > 30 * np.pi / 180:
                angle = 30 * np.pi / 180 * 2
            if angle < -30 * np.pi / 180:
                angle = -30 * np.pi / 180 * 2

            if angle >= 10 * np.pi / 180 or angle <= -10 * np.pi / 180:
                velocity = 0.8

            if angle > 20 * np.pi / 180 or angle < -20 * np.pi / 180:
                velocity = 0.3

            if angle >= -1 * np.pi / 180 and angle <= 1 * np.pi / 180:
                velocity = 2.0

            if velocity < 0:
                velocity = 1.0

            if velocity > 2.5:
                velocity = 2.5

            msg = Twist()
            msg.linear.x = velocity
            msg.angular.z = angle
            self.pub.publish(msg)

            self.publish_marker(velocity, angle)

    def publish_marker(self, velocity, angle):
        marker = Marker()
        marker.header.frame_id = "/roboworks/odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = velocity * math.cos(angle)
        marker.pose.position.y = velocity * math.sin(angle)
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

        self.marker_id += 1

def main(args=None):
    rclpy.init(args=args)

    node = PIDController()

    print("Listening to error for PID")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()