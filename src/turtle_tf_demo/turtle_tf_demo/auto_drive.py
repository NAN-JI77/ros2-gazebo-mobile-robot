import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()

        self.get_logger().info('auto_drive square started')

    def timer_callback(self):
        msg = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        cycle_time = 4.0   # 每个边+转向总时间
        move_time = 2.5    # 前进时间
        turn_time = 1.5    # 转向时间

        total_time = cycle_time * 4  # 四条边

        if elapsed < total_time:
            phase = elapsed % cycle_time

            # 前进阶段
            if phase < move_time:
                msg.linear.x = 0.3
                msg.angular.z = 0.0

            # 转向阶段
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.8

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info('auto_drive square finished')
            self.destroy_timer(self.timer)
            return

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
