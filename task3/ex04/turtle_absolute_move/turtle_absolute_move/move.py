import sys

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import math
from time import sleep

class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.take_pose,
            10)
        self.flag = True

    def take_pose(self, msg):
        #self.get_logger().info('Received:')
        #self.get_logger().info(f'   x {msg.x}')
        #self.get_logger().info(f'   y {msg.y}')
        #self.get_logger().info(f'   theta {msg.theta}')
        #self.get_logger().info('')
        #self.get_logger().info(f'   linear_velocity {msg.linear_velocity}')
        #self.get_logger().info(f'   angular_velocity {msg.angular_velocity}')
        print('take')
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.linear_velocity = msg.linear_velocity
        self.angular_velocity = msg.angular_velocity
        self.flag = False
        
    def push_pose(self, x, y, theta):
        print('push')
        while self.flag:
            rclpy.spin_once(self, timeout_sec=0.1)
        msg = Twist()
        v = (y - self.y, x - self.x)
        alpha = math.atan2(v[0], v[1])
        alpha = math.atan2(math.sin(alpha - self.theta), math.cos(alpha - self.theta))
        print(alpha)
        msg.angular.z = alpha
        self.publisher.publish(msg)
        sleep(3)
        msg = Twist()
        msg.linear.x = math.sqrt(v[0] ** 2 + v[1] ** 2)
        self.publisher.publish(msg)
        sleep(3)
        msg = Twist()
        alpha = math.atan2(math.sin(theta - alpha), math.cos(theta - alpha))
        msg.angular.z = alpha
        self.publisher.publish(msg)


def main():
    rclpy.init()
    minimal_subscriber = Mover()
    rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.push_pose(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
    minimal_subscriber.destroy_node()


if __name__ == '__main__':
    main()