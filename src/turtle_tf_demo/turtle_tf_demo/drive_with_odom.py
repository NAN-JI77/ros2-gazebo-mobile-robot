import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveWithOdom(Node):
    def __init__(self):
        super().__init__('drive_with_odom')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/simple_robot/odometry',
            self.odom_callback,
            10
        )

        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        self.target_distance = 1.0
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('drive_with_odom started')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(
                f'Start pose captured: ({self.start_x:.3f}, {self.start_y:.3f})'
            )

    def control_loop(self):
        if self.start_x is None or self.current_x is None:
            return

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        distance = math.sqrt(dx * dx + dy * dy)

        msg = Twist()

        if distance < self.target_distance:
            msg.linear.x = 0.15
            msg.angular.z = 0.0
            self.get_logger().info(
                f'distance={distance:.3f} / target={self.target_distance:.3f}'
            )
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info('Target reached, stop.')
            self.destroy_timer(self.timer)
            return

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveWithOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
