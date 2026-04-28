import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SquareWithOdom(Node):
    def __init__(self):
        super().__init__('square_with_odom')

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

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.side_length = 1.0
        self.turn_angle = math.pi / 2.0

        self.state = 'move'
        self.edge_count = 0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('square_with_odom started')

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

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw

    def reset_move_reference(self):
        self.start_x = self.current_x
        self.start_y = self.current_y

    def reset_turn_reference(self):
        self.start_yaw = self.current_yaw

    def control_loop(self):
        if self.current_x is None or self.current_yaw is None:
            return

        msg = Twist()

        if self.edge_count >= 4:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info('Square finished')
            self.destroy_timer(self.timer)
            return

        if self.state == 'move':
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < self.side_length:
                msg.linear.x = 0.15
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.state = 'turn'
                self.reset_turn_reference()
                self.get_logger().info(f'Edge {self.edge_count + 1} finished, start turning')

        elif self.state == 'turn':
            turned = self.normalize_angle(self.current_yaw - self.start_yaw)

            if turned < self.turn_angle:
                msg.linear.x = 0.0
                msg.angular.z = 0.3
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.edge_count += 1
                self.state = 'move'
                self.reset_move_reference()
                self.get_logger().info(f'Turn {self.edge_count} finished, start next edge')

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquareWithOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
