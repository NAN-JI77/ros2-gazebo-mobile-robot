import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/simple_robot/odometry',
            self.odom_callback,
            10
        )

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # 目标点
        self.goal_x = 1.0
        self.goal_y = 1.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'go_to_goal started, target=({self.goal_x}, {self.goal_y})')

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
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def control_loop(self):
        if self.current_x is None:
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.sqrt(dx * dx + dy * dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

        msg = Twist()

        # 到达目标点
        if distance < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info('Goal reached, stop.')
            self.destroy_timer(self.timer)
            return

        # 先转向
        if abs(yaw_error) > 0.2:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 if yaw_error > 0 else -0.5

        # 再前进
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'pos=({self.current_x:.2f}, {self.current_y:.2f}) '
            f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'dist={distance:.2f} yaw_error={yaw_error:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
