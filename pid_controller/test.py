import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class MapToGPSListener(Node):
    def __init__(self):
        super().__init__('map_to_gps_listener')

        # Create a tf2_ros.Buffer and a tf2_ros.TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a timer to periodically check the transform
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Look up the transform
            transform = self.tf_buffer.lookup_transform('roboworks/odom', 'roboworks/lidar_link', rclpy.time.Time())
            
            # Print the transform
            self.get_logger().info('Transform: ' + str(transform))

        except Exception as e:
            # If the transform is not available, log the error
            self.get_logger().error('Could not get transform: ' + str(e))

def main(args=None):
    rclpy.init(args=args)

    # Create a MapToGPSListener node
    node = MapToGPSListener()

    # Spin the node so it can process callbacks
    rclpy.spin(node)

    # Shutdown and cleanup the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()