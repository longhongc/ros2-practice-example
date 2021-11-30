import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import signal
from data_processor.utils import signal_handler, plot


class SpeedSubscriber(Node):

    def __init__(self):
        super().__init__('speed_subscriber')
        self.subscription = self.create_subscription(Twist, 'turtle1/cmd_vel', self.listener_callback, 10)
        self.subscription #prevent unused variable warning
        self.count = 0
        plot.speed_plot_setting()

    def listener_callback(self, msg):
        plot.draw_speed(self.count, msg.linear.x, msg.angular.z)
        self.count+=1
        
        lx = msg.linear.x
        ly = msg.linear.y
        lz = msg.linear.z
        ax = msg.angular.x
        ay = msg.angular.y
        az = msg.angular.z

        self.get_logger().info('I heard: speed '+ \
                               'linear: [x: %lf, y: %lf, z, %lf] ' \
                               'angular: [x: %lf, y: %lf, z: %lf]'%(lx, ly, lz, ax, ay, az))



def main(args=None):
    signal.signal(signal.SIGINT, signal_handler.signal_handler)
    rclpy.init(args=args)

    speed_subscriber = SpeedSubscriber()
    rclpy.spin(speed_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically) 
    speed_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
