#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

class RangeSensorPublisher(Node):

    def __init__(self):
        super().__init__('range_sensor_publisher')
        
        # Create a subscriber to the Range topic
        self.subscription = self.create_subscription(
            Range,
            'ultrasonic_sensor',  # Replace with your actual topic name
            self.range_callback,
            10)

    def range_callback(self, msg):
        distance = msg.range
        self.get_logger().info(f'Distance: {distance} meters')


def main(args=None):
    rclpy.init(args=args)
    range_sensor_publisher = RangeSensorPublisher()
    rclpy.spin(range_sensor_publisher)
    
    # Shutdown
    range_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
