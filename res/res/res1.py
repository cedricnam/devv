import rclpy
from std_msgs.msg import String
import time

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('nam_node')

    publisher = node.create_publisher(String, 'nam_topic', 10)

    def callback(msg):
        if msg.data == "res":
            node.destroy_node()
            restart_program()

    subscriber = node.create_subscription(String, 'kill', callback, 10)

    msg = String()
    msg.data = "Nam"

    timer = node.create_timer(1.0, lambda: publisher.publish(msg))

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

def restart_program():
    print("Restarting program...")
    time.sleep(1)
    main()

if __name__ == '__main__':
    main()