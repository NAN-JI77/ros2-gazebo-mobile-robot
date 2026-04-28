import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WaypointAvoid(Node):
    def __init__(self):
        super().__init__('waypoint_avoid')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/simple_robot/odometry',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 当前位姿
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # 雷达距离
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.turn_direction = 1.0

        # waypoint 路线
        self.goals = [
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 2.0),
            (1.0, 2.0),
        ]
        self.goal_index = 0

        # 平滑巡航参数
        self.k_linear = 0.6
        self.k_angular = 1.2
        self.max_linear = 0.16
        self.max_angular = 0.6
        self.goal_tolerance = 0.18

        # 避障参数
        self.danger_distance = 0.30
        self.safe_distance = 0.5

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('waypoint_avoid started')

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

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def normalize_range(self, r, range_min, range_max):
        if math.isnan(r) or math.isinf(r):
            return None
        if r <= range_min or r >= range_max:
            return None
        return r

    def sector_min(self, msg: LaserScan, min_angle_deg, max_angle_deg):
        values = []

        min_angle = math.radians(min_angle_deg)
        max_angle = math.radians(max_angle_deg)

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if min_angle <= angle <= max_angle:
                rr = self.normalize_range(r, msg.range_min, msg.range_max)
                if rr is not None:
                    values.append(rr)

        if len(values) == 0:
            return float('inf')

        return min(values)

    def scan_callback(self, msg: LaserScan):
        # 正前方
        self.front_distance = self.sector_min(msg, -20, 20)

        # 左前方
        self.left_distance = self.sector_min(msg, 30, 100)

        # 右前方
        self.right_distance = self.sector_min(msg, -100, -30)

        if self.left_distance >= self.right_distance:
            self.turn_direction = 1.0
        else:
            self.turn_direction = -1.0

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def obstacle_control(self):
        msg = Twist()

        # 很近：先后退并转向
        if self.front_distance < self.danger_distance:
            msg.linear.x = -0.05
            msg.angular.z = 0.65 * self.turn_direction

            self.get_logger().warn(
                f'DANGER avoid: front={self.front_distance:.2f}, '
                f'left={self.left_distance:.2f}, right={self.right_distance:.2f}'
            )

        # 不太近：慢速绕开
        else:
            msg.linear.x = 0.07
            msg.angular.z = 0.40 * self.turn_direction

            self.get_logger().info(
                f'Avoiding: front={self.front_distance:.2f}, '
                f'left={self.left_distance:.2f}, right={self.right_distance:.2f}'
            )

        self.cmd_pub.publish(msg)

    def waypoint_control(self):
        msg = Twist()

        if self.goal_index >= len(self.goals):
            self.publish_stop()
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
        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.goal_index + 1}: ({goal_x:.2f}, {goal_y:.2f})'
            )
            self.goal_index += 1
            self.publish_stop()
            return

        # 平滑控制
        linear_speed = self.k_linear * distance
        angular_speed = self.k_angular * yaw_error

        # 角度偏差大时，降低线速度
        if abs(yaw_error) > 0.6:
            linear_speed *= 0.3
        elif abs(yaw_error) > 0.3:
            linear_speed *= 0.6

        linear_speed = self.clamp(linear_speed, 0.0, self.max_linear)
        angular_speed = self.clamp(angular_speed, -self.max_angular, self.max_angular)

        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'Waypoint {self.goal_index + 1}/{len(self.goals)} '
            f'pos=({self.current_x:.2f}, {self.current_y:.2f}) '
            f'target=({goal_x:.2f}, {goal_y:.2f}) '
            f'dist={distance:.2f} yaw_error={yaw_error:.2f} '
            f'front={self.front_distance:.2f}'
        )

    def control_loop(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        # 优先级：避障 > 巡航
        if self.front_distance < self.safe_distance:
            self.obstacle_control()
        else:
            self.waypoint_control()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
