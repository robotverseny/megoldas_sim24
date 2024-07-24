
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.msg import PidState
import math
import numpy as np

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.pub = self.create_publisher(Twist, 'roboworks/cmd_vel', 1)

        self.kp = 9.5
        self.kd = 0.5
        self.kp_vel = 42.0
        self.kd_vel = 0.0
        self.ki = 0.0
        self.servo_offset = 18.0*math.pi/180
        self.prev_error = 0.0
        self.error = 0.0
        self.vel_input = 1.0

        self.subscription = self.create_subscription(
            PidState,
            'error',
            self.control,1)
        

    def control(self, data):
        self.error = 5*data.p_error

        if self.error != 0.0:
            control_error = self.kp*self.error + self.kd*(self.error - self.prev_error)
            angle = self.servo_offset + control_error*np.pi/180
            #angle=np.clip(angle, -1, 1)
            control_error_vel = self.kp_vel*self.error + self.kd_vel*(self.error - self.prev_error)
            velocity = data.p_term + abs(control_error_vel)

            self.prev_error = self.error

            if angle > 30*np.pi/180:
                angle = 30*np.pi/180 *2
            if angle < -30*np.pi/180:
                angle = -30*np.pi/180 *2

            if angle >= 10*np.pi/180 or angle <= -10*np.pi/180:
                velocity = 0.8

            if angle > 20*np.pi/180 or angle < -20*np.pi/180:
                velocity = 0.3

            if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
                velocity = 2.0

            if velocity < 0:
                velocity = 1

            if velocity > 2.5:
                velocity = 2.5

            msg = Twist()
            msg.linear.x = velocity
            msg.angular.z = angle
            self.pub.publish(msg)
           

def main(args=None):
    rclpy.init(args=args)

    node = PIDController()

    print("Listening to error for PID")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()