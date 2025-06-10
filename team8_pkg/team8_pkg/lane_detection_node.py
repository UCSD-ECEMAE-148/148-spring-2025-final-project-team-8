import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray
from custom_interfaces.msg import LabeledCentroid
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

import statistics

# Nodes in this program
NODE_NAME = 'lane_detection_node'

# Topics subcribed/published to in this program
CAMERA_TOPIC_NAME = '/centroids'
CENTROID_TOPIC_NAME = '/location'
LIDAR_TOPIC_NAME = '/scan'
LABEL = 1
CAM_CENTER = 0.5

# DISTANCE CALCULATION CONST
ONE_FT_PXL_DIAMETER = 150               # diameter of the object from 1 ft
DISTANCE_FUNC_COEF = 12 * ONE_FT_PXL_DIAMETER     # store coefficient to reduce calculations

class LaneDetection(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_error_publisher = self.create_publisher(Float32MultiArray, CENTROID_TOPIC_NAME, 10)
        self.centroid_error_publisher
        self.centroid_error = Float32MultiArray()
        self.camera_subscriber = self.create_subscription(LabeledCentroid, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.camera_subscriber
        self.lidar_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.update_ranges, 10)
        self.lidar_subscriber
        self.ranges = None
        self.N = 0
        self.n = 0

    def locate_centroid(self, data):
        try:
            # When centroid with our label gets detected
            if data.label == LABEL:
                mid_x = data.cx     #mid_x is a percentage of screen
                self.ek = float(mid_x - CAM_CENTER)
                lidar_center = int(2*self.n*(1 - mid_x))
                #self.get_logger().info(f"lidar center: {lidar_center}")
                #self.get_logger().info(f"{len(self.ranges)}")
                distance = min(self.ranges[lidar_center-3: lidar_center+3])

                #self.get_logger().info(f"Found a guy: [tracking error: {self.ek}], [label: {data.label}] [distance prediction (m): {distance}]")

                # publish error data
                self.centroid_error.data = [float(self.ek), distance]
                self.centroid_error_publisher.publish(self.centroid_error)

            # When Nothing was found
            else:
                self.get_logger().info(f"Nothing detected")
        except ValueError:
            pass

        # plotting results
        #self.debug_cv = self.get_parameter('debug_cv').value # ability to update debug in real-time
        #if self.debug_cv:
        #    cv2.imshow('img', img)
        #    cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
        #    cv2.waitKey(1)
        #else:
        #    cv2.destroyAllWindows()
    
    def update_ranges(self, data):
        if self.N == 0 and self.n == 0:
            self.N = len(data.ranges)
            self.n = int((self.N / 360.0) * 35)
        self.ranges = data.ranges[-self.n:] + data.ranges[:self.n]


def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = LaneDetection()
    try:
        rclpy.spin(centroid_publisher)
        centroid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        centroid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        centroid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
