#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera_rgb/image_raw',  # Replace with your camera topic
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.twist = Twist()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Threshold the image to get only black colors
        _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        # Find the contours of the black line
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If no contours are found, stop the robot
        if len(contours) == 0:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            return

        # Find the largest contour and its center
        contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Display the image for debugging
        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

        # Control logic: if the line is to the left or right of the center
        err = cx - cv_image.shape[1] / 2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100  # Adjust the denominator for sensitivity
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
