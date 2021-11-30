import rclpy
from rclpy.node import Node

from sensor_interfaces.msg import Temperature

import signal
from data_processor.utils import signal_handler, plot


class TemperatureSubscriber(Node):

    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(Temperature, 'temp', self.listener_callback, 10)
        self.subscription #prevent unused variable warning
        self.count = 0
        plot.temperature_plot_setting()

    def listener_callback(self, msg):
        plot.draw_temp(self.count, msg.temp)
        self.count+=1
        self.get_logger().info('I heard: %s' % msg.temp)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler.signal_handler)
    rclpy.init(args=args)

    temperature_subscriber = TemperatureSubscriber()
    rclpy.spin(temperature_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically) 
    temperature_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
