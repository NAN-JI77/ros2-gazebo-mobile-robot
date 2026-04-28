import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TurnWithOdom(Node):
    def __init__(self):
        super().__init__('turn_with_odom')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/simple_robot/odometry',
            self.odom_callback,
            10
        )

        self.start_yaw = None
        self.current_yaw = None

        self.target_angle = math.pi / 2.0   # 左转 90 度
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('turn_with_odom started')

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation

        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            self.get_logger().info(f'Start yaw captured: {self.start_yaw:.3f} rad')

    def control_loop(self):
        if self.start_yaw is None or self.current_yaw is None:
            return

        angle_turned = self.normalize_angle(self.current_yaw - self.start_yaw)

        msg = Twist()

        if angle_turned < self.target_angle:
            msg.linear.x = 0.0
            msg.angular.z = 0.3
            self.get_logger().info(
                f'angle_turned={angle_turned:.3f} / target={self.target_angle:.3f}'
            )
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info('Target angle reached, stop.')
            self.destroy_timer(self.timer)
            return

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurnWithOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
