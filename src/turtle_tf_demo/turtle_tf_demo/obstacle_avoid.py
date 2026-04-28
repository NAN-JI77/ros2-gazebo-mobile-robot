import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # 距离阈值
        self.danger_distance = 0.35   # 太近：后退+转向
        self.safe_distance = 0.90     # 小于这个：开始绕开

        # 默认转向方向
        self.turn_direction = 1.0     # 1.0 左转，-1.0 右转

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('obstacle_avoid started')

    def normalize_range(self, r, range_min, range_max):
        if math.isnan(r):
            return None
        if math.isinf(r):
            return None
        if r <= range_min:
            return None
        if r >= range_max:
            return None
        return r

    def sector_min(self, msg: LaserScan, min_angle_deg, max_angle_deg):
        """
        根据角度范围取最小距离。
        这里不用假设 ranges 的中间就是正前方，而是用 angle_min 和 angle_increment 计算。
        """
        ranges = msg.ranges
        values = []

        min_angle = math.radians(min_angle_deg)
        max_angle = math.radians(max_angle_deg)

        for i, r in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if min_angle <= angle <= max_angle:
                rr = self.normalize_range(r, msg.range_min, msg.range_max)
                if rr is not None:
                    values.append(rr)

        if len(values) == 0:
            return float('inf')

        return min(values)

    def scan_callback(self, msg: LaserScan):
        # 正前方：-20° 到 20°
        self.front_distance = self.sector_min(msg, -20, 20)

        # 左前方：30° 到 100°
        self.left_distance = self.sector_min(msg, 30, 100)

        # 右前方：-100° 到 -30°
        self.right_distance = self.sector_min(msg, -100, -30)

        # 哪边更空，之后就往哪边转
        if self.left_distance >= self.right_distance:
            self.turn_direction = 1.0
        else:
            self.turn_direction = -1.0

    def control_loop(self):
        msg = Twist()

        # 情况 1：非常近，已经快撞上了，先后退并转向
        if self.front_distance < self.danger_distance:
            msg.linear.x = -0.05
            msg.angular.z = 0.6 * self.turn_direction

            self.get_logger().warn(
                f'DANGER: front={self.front_distance:.2f}m, '
                f'left={self.left_distance:.2f}m, '
                f'right={self.right_distance:.2f}m, '
                f'backing and turning'
            )

        # 情况 2：前方有障碍，但还没特别近，慢速边走边绕
        elif self.front_distance < self.safe_distance:
            msg.linear.x = 0.04
            msg.angular.z = 0.55 * self.turn_direction

            self.get_logger().info(
                f'Obstacle: front={self.front_distance:.2f}m, '
                f'left={self.left_distance:.2f}m, '
                f'right={self.right_distance:.2f}m, '
                f'slow turning'
            )

        # 情况 3：前方安全，正常前进
        else:
            msg.linear.x = 0.12
            msg.angular.z = 0.0

            self.get_logger().info(
                f'Clear: front={self.front_distance:.2f}m, moving forward'
            )

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
