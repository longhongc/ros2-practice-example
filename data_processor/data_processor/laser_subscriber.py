import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import signal
from data_processor.utils import signal_handler, plot


class LaserSubscriber(Node):

    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec

        # print the received laser timestamp to examine subscriber
        self.get_logger().info('I heard: laser %d.%ld'%(sec, nsec))


def main(args=None):
    # Ctrl-c to terminate the node
    signal.signal(signal.SIGINT, signal_handler.signal_handler)
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()
    rclpy.spin(laser_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically) 
    laser_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
