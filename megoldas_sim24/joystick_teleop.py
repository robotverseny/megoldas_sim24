import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickTeleopNode(Node):
    def __init__(self):
        super().__init__('joystick_teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'roboworks/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.linear_axis = 1  # Typically the left stick vertical axis
        self.angular_axis = 0  # Typically the left stick horizontal axis
        self.linear_scale = 1.0  # Scale for linear speed
        self.angular_scale = 1.0  # Scale for angular speed
        self.get_logger().info('Joystick Teleop Node initialized.')

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale
        twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()