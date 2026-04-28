import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class FollowWaypoints(Node):
    def __init__(self):
        super().__init__('follow_waypoints')

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

        # 多个目标点
        self.goals = [
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 2.0),
            (1.0, 2.0),
        ]
        self.goal_index = 0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'follow_waypoints started, goals={self.goals}')

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

        msg = Twist()

        # 所有目标点完成
        if self.goal_index >= len(self.goals):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info('All waypoints reached, stop.')
            self.destroy_timer(self.timer)
            return

        goal_x, goal_y = self.goals[self.goal_index]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y

        distance = math.sqrt(dx * dx + dy * dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

        # 到达当前目标点
        if distance < 0.15:
            self.get_logger().info(
                f'Reached waypoint {self.goal_index + 1}: ({goal_x:.2f}, {goal_y:.2f})'
            )
            self.goal_index += 1
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        # 先转向
        if abs(yaw_error) > 0.2:
            msg.linear.x = 0.0
            msg.angular.z = 0.3 if yaw_error > 0 else -0.3

        # 再前进
        else:
            msg.linear.x = 0.15
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'goal_{self.goal_index + 1}/{len(self.goals)} '
            f'current=({self.current_x:.2f}, {self.current_y:.2f}) '
            f'target=({goal_x:.2f}, {goal_y:.2f}) '
            f'dist={distance:.2f} yaw_error={yaw_error:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
