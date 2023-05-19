import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import time
import os
import sys

sys.path.append('./src/devv/reta/reta')
from database import GimbalDatabase
kill_it = False

class Helo(Node):
    def __init__(self):
        super().__init__('helo')
        self.signal = self.create_subscription(
            String,
            'kill',
            self.listener_callback,
            10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        db = GimbalDatabase()
        self._serial_port = db.serial_port

    def timer_callback(self):
        msg = String()
        msg.data = os.getcwd() + ' ' + self._serial_port
        self.get_logger().info(msg.data)

    def listener_callback(self, msg):
        global kill_it
        kill_it = True
        print("audman")

def main(args=None):
    global kill_it
    rclpy.init(args=args)
    print('hehehehehhehe')
    helo = Helo()
    while rclpy.ok():
        rclpy.spin_once(helo)
        if kill_it == True:
            helo.destroy_node()
            rclpy.shutdown()
            kill_it = False
            main()

if __name__ == '__main__':
    main()
