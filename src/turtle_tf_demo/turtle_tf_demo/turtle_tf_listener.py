import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, TransformException


class TurtleTFListener(Node):
    def __init__(self):
        super().__init__('turtle_tf_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('turtle_tf_listener started')

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',   # target frame
                'turtle1', # source frame
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            self.get_logger().info(
                f'turtle1 in world: x={x:.2f}, y={y:.2f}, z={z:.2f}, '
                f'qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}'
            )

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform world->turtle1: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
