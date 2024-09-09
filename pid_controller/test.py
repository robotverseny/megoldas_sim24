import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan

class SimplePursuit(Node):
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sub = self.create_subscription(LaserScan, 'roboworks/scan', self.callbackLaser, 1)
    def timer_callback(self):
        if self.first_run:
            try:
                self.trans = self.buf.lookup_transform("roboworks/scan", "roboworks/lidar_link", rclpy.time.Time())
                print(self.trans)
                self.get_logger().info("Got laser transform")
                self.first_run = False
            except tf2_ros.TransformException as ex:
                self.get_logger().info(f'Could not transform {"roboworks/odom"} to {"roboworks/lidar_link"}: {ex}')
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