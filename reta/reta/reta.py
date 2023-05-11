import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Killer(Node):
    def __init__(self):
        super().__init__('Killer')
        self.publisher_ = self.create_publisher(String, 'kill', 10)
        msg = String()
        msg.data = 'res'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    killer = Killer()
    rclpy.spin(killer)
    killer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()