import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WaypointAvoidSM(Node):
    def __init__(self):
        super().__init__('waypoint_avoid_sm')

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
        self.safe_distance = 0.5
        self.danger_distance = 0.35

        # 状态机参数
        self.state = 'NORMAL'
        self.state_start_time = self.get_clock().now()

        self.backup_duration = 0.7
        self.turn_duration = 2.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('waypoint_avoid_sm started')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().warn(f'STATE -> {new_state}')

    def state_elapsed(self):
        now = self.get_clock().now()
        return (now - self.state_start_time).nanoseconds / 1e9

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

    def odom_callback(self, msg):
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

    def sector_min(self, msg, min_angle_deg, max_angle_deg):
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

    def scan_callback(self, msg):
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

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    def normal_waypoint_control(self):
        if self.goal_index >= len(self.goals):
            self.publish_stop()
            self.get_logger().info('All waypoints reached, stop.')
            self.destroy_timer(self.timer)
            return

        # 检测到障碍，进入后退状态
        if self.front_distance < self.safe_distance:
            self.get_logger().warn(
                f'Obstacle detected: front={self.front_distance:.2f}, '
                f'left={self.left_distance:.2f}, right={self.right_distance:.2f}'
            )
            self.set_state('BACKUP')
            return

        goal_x, goal_y = self.goals[self.goal_index]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y

        distance = math.sqrt(dx * dx + dy * dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.goal_index + 1}: ({goal_x:.2f}, {goal_y:.2f})'
            )
            self.goal_index += 1
            self.publish_stop()
            return

        linear_speed = self.k_linear * distance
        angular_speed = self.k_angular * yaw_error

        if abs(yaw_error) > 0.6:
            linear_speed *= 0.3
        elif abs(yaw_error) > 0.3:
            linear_speed *= 0.6

        linear_speed = self.clamp(linear_speed, 0.0, self.max_linear)
        angular_speed = self.clamp(angular_speed, -self.max_angular, self.max_angular)

        self.publish_cmd(linear_speed, angular_speed)

        self.get_logger().info(
            f'NORMAL waypoint {self.goal_index + 1}/{len(self.goals)} '
            f'pos=({self.current_x:.2f}, {self.current_y:.2f}) '
            f'target=({goal_x:.2f}, {goal_y:.2f}) '
            f'dist={distance:.2f} yaw_error={yaw_error:.2f} '
            f'front={self.front_distance:.2f}'
        )

    def backup_control(self):
        elapsed = self.state_elapsed()

        if elapsed < self.backup_duration:
            # 后退，同时略微往更空的一侧转
            self.publish_cmd(-0.06, 0.25 * self.turn_direction)
            self.get_logger().warn(
                f'BACKUP elapsed={elapsed:.2f}, front={self.front_distance:.2f}'
            )
        else:
            self.set_state('TURN')

    def turn_control(self):
        elapsed = self.state_elapsed()

        if elapsed < self.turn_duration:
            # 原地转向
            self.publish_cmd(0.0, 0.65 * self.turn_direction)
            self.get_logger().warn(
                f'TURN elapsed={elapsed:.2f}, direction={self.turn_direction}'
            )
        else:
            self.set_state('NORMAL')

    def control_loop(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if self.state == 'NORMAL':
            self.normal_waypoint_control()

        elif self.state == 'BACKUP':
            self.backup_control()

        elif self.state == 'TURN':
            self.turn_control()

        else:
            self.publish_stop()
            self.set_state('NORMAL')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointAvoidSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
