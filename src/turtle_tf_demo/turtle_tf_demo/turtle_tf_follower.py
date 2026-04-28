import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


class TurtleTFFollower(Node):
    def __init__(self):
        super().__init__('turtle_tf_follower')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.get_logger().info('turtle_tf_follower started')

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'turtle2',
                'turtle1',
                rclpy.time.Time()
            )

            dx = transform.transform.translation.x
            dy = transform.transform.translation.y

            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)

            cmd = Twist()
            cmd.linear.x = 1.0 * distance
            cmd.angular.z = 4.0 * angle_to_target

            if cmd.linear.x > 2.0:
                cmd.linear.x = 2.0
            if cmd.angular.z > 2.0:
                cmd.angular.z = 2.0
            if cmd.angular.z < -2.0:
                cmd.angular.z = -2.0

            self.cmd_pub.publish(cmd)

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform turtle2->turtle1: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
