import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import numpy as np
from math import atan2, asin

class ControllerSubscriber(Node):

    def __init__(self):
        super().__init__('controller_subscriber')

        self.subscription = self.create_subscription(
            Pose,
            'setpoint_pose',
            self.listener_callback,
            10)
        self.subscription

        self.publisher = self.create_publisher(
            Twist, 
            "setpoint_euler", 
            10)

    def listener_callback(self, msg):

        pos = np.array([msg.position.x, msg.position.y, msg.position.z, 0], dtype='float64').transpose()
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w] 

        l1 = [1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2], 0]
        l2 = [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1], 0]
        l3 = [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2, 0]
        l4 = [0, 0, 0, 1]

        matrix =  np.array([l1, l2, l3, l4], dtype='float64')

        new_pos = matrix @ pos


        #self.get_logger().info('I heard: "%s"' % msg.position)
        print("rotation matrix")
        print(matrix)

        # publish
        twist_msg = Twist()

        twist_msg.linear.x = pos[0]
        twist_msg.linear.y = pos[1]
        twist_msg.linear.z = pos[2]

        roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*q[1]**2 - 2*q[2]**2)
        pitch = asin(2*q[0]*q[2] - 2*q[1]*q[3])
        yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*q[2]**2 - 2*q[2]**2)

        twist_msg.angular.x = roll
        twist_msg.angular.y = pitch
        twist_msg.angular.z = yaw

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    controller_subscriber = ControllerSubscriber()

    rclpy.spin(controller_subscriber)

    controller_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()