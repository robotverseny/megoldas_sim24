import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from control_msgs.msg import PidState
import math
import std_msgs.msg

class DistFinder(Node):
    def __init__(self):
        super().__init__('dist_finder')
        self.activated_ = True
        self.publisher_error = self.create_publisher(PidState, 'error', 10)
        self.publisher_pid_data = self.create_publisher(std_msgs.msg.String, 'pid_data', 10)
        self.publisher_kozepiskola = self.create_publisher(std_msgs.msg.String, 'kozepiskola', 10)
        self.subscription = self.create_subscription(LaserScan, '/roboworks/scan', self.laser_callback, 10)
    
        self.KOZEPISKOLA_NEVE = "Ismeretlen kozepiskola"
        self.KOZEPISKOLA_AZON = "A00"
        self.ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
        self.DESIRED_DISTANCE_RIGHT = 1.0  # 0.9 meters
        self.DESIRED_DISTANCE_LEFT = 0.8  # 0.55
        self.VELOCITY = 1.00  # meters per second
        self.CAR_LENGTH = 0.50  # 0.5 meters
        # self.message = std_msgs.msg.String()
        # self.message.data = self.KOZEPISKOLA_NEVE + "(" + self.KOZEPISKOLA_AZON + ")"
        # self.publisher_kozepiskola.publish(self.message)
        self.get_logger().info('init')

    
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        if angle > 179.9:
            angle = 179.9
        index = len(data.ranges) * (angle + 45) / self.ANGLE_RANGE
        dist = data.ranges[int(index)]
        if math.isinf(dist):
            return 10.0
        if math.isnan(dist):
            return 4.0
        return data.ranges[int(index)]

    def followRight(self, data, desired_trajectory):
        # data: single message from topic /scan
        # desired_trajetory: desired distance to the right wall [meters]
        messageS1 = std_msgs.msg.String()
        messageS1.data = "Jobb oldal kovetes"
        self.get_logger().info('follow right')

        a = self.getRange(data, 60)
        b = self.getRange(data, 0)
        swing = math.radians(60)
        alpha = math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
        messageS1.data += "\na: %.1f b: %.1f" % (a, b)
        messageS1.data += "\nAlpha left %.1f" % (math.degrees(alpha))
        curr_dist = b * math.cos(alpha)

        future_dist = curr_dist + self.CAR_LENGTH * math.sin(alpha)
        messageS1.data += "\nRight: %.2f" % (future_dist)
        error = desired_trajectory - future_dist

        messageS1.data += "\nCurrent Distance Left: %.2f" % (curr_dist)
        print(messageS1)
        self.publisher_pid_data.publish(messageS1)
        return error, curr_dist
    def followLeft(self, data, desired_trajectory):
        # data: single message from topic /scan
        # desired_trajectory: desired distance to the left wall [meters]
        messageS1 = std_msgs.msg.String()
        a = self.getRange(data, 120)
        b = self.getRange(data, 179.9)
        swing = math.radians(60)
        messageS1.data = "Bal oldal kovetes\na: %.1f b: %.1f" % (a, b)
        alpha = -math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
        messageS1.data += "\nAlpha left %.1f" % (math.degrees(alpha))
        curr_dist = b * math.cos(alpha)
        future_dist = curr_dist - self.CAR_LENGTH * math.sin(alpha)
        messageS1.data += "\nLeft: %.2f" % (future_dist)
        error = future_dist - desired_trajectory
        messageS1.data += "\nCurrent Distance Left: %.2f" % (curr_dist)
        self.publisher_pid_data.publish(messageS1)
        return error, curr_dist
    def followCenter(self, data):
        # data: single message from topic /scan
        messageS1 = std_msgs.msg.String()
        messageS1.data = "Kozepvonal kovetes"
        self.get_logger().info('followcenter')

        a = self.getRange(data, 120)
        b = self.getRange(data, 179.9)
        swing = math.radians(60)
        alpha = -math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
        curr_dist1 = b * math.cos(alpha)
        future_dist1 = curr_dist1 - self.CAR_LENGTH * math.sin(alpha)

        a = self.getRange(data, 60)
        b = self.getRange(data, 0)
        swing = math.radians(60)
        alpha = math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
        curr_dist2 = b * math.cos(alpha)
        future_dist2 = curr_dist2 + self.CAR_LENGTH * math.sin(alpha)

        error = future_dist1 - future_dist2
        messageS1.data += "\nError: %.2f" % error
        self.publisher_kozepiskola.publish(messageS1)
        return error, curr_dist2 - curr_dist1
    
    def laser_callback(self, data):
        self.get_logger().info('callback')

        # Does a left wall follow
        #error_left, curr_dist_left = followLeft(data, DESIRED_DISTANCE_LEFT)
        #error = error_left
        global error
        error_right, curr_dist_right = self.followRight(data, self.DESIRED_DISTANCE_RIGHT)
        error = error_right

        # This is code bock for center wall follow
        #error_center, curr_dist_center = followCenter(data)
        #error = error_center
        
        msg = PidState()
        msg.p_error = error
        msg.p_term = self.VELOCITY
        self.publisher_error.publish(msg)

def main(args=None):
    rclpy.init(args=args) 
    node = DistFinder()
    node.get_logger().info('Laser node started')
    rclpy.spin(node)
    rate = node.create_rate(2)  # 2hz
    while rclpy.ok():
        node.get_logger().info('ok')

        message = std_msgs.msg.String()
        message.data = node.KOZEPISKOLA_NEVE + "(" + node.KOZEPISKOLA_AZON + ")"
        node.publisher_kozepiskola.publish(message)
        rate.sleep()
    print("ok")    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()