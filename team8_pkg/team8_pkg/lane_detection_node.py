import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

import statistics

# Nodes in this program
NODE_NAME = 'lane_detection_node'

# Topics subcribed/published to in this program
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/location'
LIDAR_TOPIC_NAME = '/scan'

# DISTANCE CALCULATION CONST
ONE_FT_PXL_DIAMETER = 150               # diameter of the object from 1 ft
DISTANCE_FUNC_COEF = 12 * ONE_FT_PXL_DIAMETER     # store coefficient to reduce calculations

class LaneDetection(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_error_publisher = self.create_publisher(Float32MultiArray, CENTROID_TOPIC_NAME, 10)
        self.centroid_error_publisher
        self.centroid_error = Float32MultiArray()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.camera_subscriber
        self.lidar_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.update_ranges, 10)
        self.lidar_subscriber
        self.ranges = None
        self.N = 0
        self.n = 0

        self.bridge = CvBridge()
        self.max_num_lines_detected = 10
        self.image_width = 0
        self.image_height = 0
        self.start_height = 0
        self.bottom_height = 0
        self.left_width = 0
        self.right_width = 0
        self.error_threshold = 0.1
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Hue_low', 1),
                ('Hue_high', 1),
                ('Saturation_low', 1),
                ('Saturation_high', 1),
                ('Value_low', 1),
                ('Value_high', 1),
                ('gray_lower', 1),
                ('inverted_filter', 0),
                ('kernal_size',1),
                ('erosion_itterations',1),
                ('dilation_itterations',1),
                ('number_of_lines', 0),
                ('error_threshold', 0),
                ('Width_min', 1),
                ('Width_max', 1),
                ('crop_width_decimal',0.8),
                ('rows_to_watch_decimal',0.2),
                ('rows_offset_decimal',0.5),
                ('camera_centerline',0.5),
                ('debug_cv', 0)
            ])
        self.Hue_low = self.get_parameter('Hue_low').value
        self.Hue_high = self.get_parameter('Hue_high').value
        self.Saturation_low = self.get_parameter('Saturation_low').value
        self.Saturation_high = self.get_parameter('Saturation_high').value
        self.Value_low = self.get_parameter('Value_low').value
        self.Value_high = self.get_parameter('Value_high').value
        self.gray_lower = self.get_parameter('gray_lower').value
        self.inverted_filter = self.get_parameter('inverted_filter').value
        self.kernal_size = self.get_parameter('kernal_size').value
        self.erosion_itterations = self.get_parameter('erosion_itterations').value
        self.dilation_itterations = self.get_parameter('dilation_itterations').value
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.min_width = self.get_parameter('Width_min').value
        self.max_width = self.get_parameter('Width_max').value
        self.crop_width_decimal = self.get_parameter('crop_width_decimal').value
        self.rows_to_watch_decimal = self.get_parameter('rows_to_watch_decimal').value
        self.rows_offset_decimal = self.get_parameter('rows_offset_decimal').value
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.debug_cv = self.get_parameter('debug_cv').value
        
        self.camera_init = False
        
        self.get_logger().info(
            f'\nHue_low: {self.Hue_low}'
            f'\nHue_high: {self.Hue_high}'
            f'\nSaturation_low: {self.Saturation_low}'
            f'\nSaturation_high: {self.Saturation_high}'
            f'\nValue_low: {self.Value_low}'
            f'\nValue_high: {self.Value_high}'
            f'\ngray_lower: {self.gray_lower}'
            f'\ninverted_filter: {self.inverted_filter}'
            f'\nkernal_size: {self.kernal_size}'
            f'\nerosion_itterations: {self.erosion_itterations}'
            f'\ndilation_itterations: {self.dilation_itterations}'
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}'
            f'\ncrop_width_decimal: {self.crop_width_decimal}'
            f'\nrows_to_watch_decimal: {self.rows_to_watch_decimal}'
            f'\nrows_offset_decimal: {self.rows_offset_decimal}'
            f'\ncamera_centerline: {self.camera_centerline}'
            f'\ndebug_cv: {self.debug_cv}')


    def locate_centroid(self, data):
        # Image processing from rosparams
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        #self.get_logger().info(f"Frame: {frame}")
        if not self.camera_init:
            self.get_logger().info(f'\n Initializing Camera...')
            height, width, channels = frame.shape
        
            # Vertical crop/pan
            rows_to_watch = int(height * self.rows_to_watch_decimal)
            rows_offset = int(height * (1 - self.rows_offset_decimal))

            # Horizontal crop
            self.start_height = int(height - rows_offset)
            self.bottom_height = int(self.start_height + rows_to_watch)
            self.left_width = int((width / 2) * (1 - self.crop_width_decimal))
            self.right_width = int((width / 2) * (1 + self.crop_width_decimal))
            self.camera_init = True
            self.get_logger().info(f'\n Camera Initialized')

        self.image_width = int(self.right_width - self.left_width)
        self.image_height = self.bottom_height-self.start_height
        img = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([self.Hue_low, self.Saturation_low, self.Value_low])
        upper = np.array([self.Hue_high, self.Saturation_high, self.Value_high])
        mask = cv2.inRange(hsv, lower, upper)

        if self.inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=mask)

        # changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        
        # get rid of white noise from grass
        kernel = np.ones((self.kernal_size, self.kernal_size), np.uint8)
        blurred = cv2.blur(blackAndWhiteImage,(self.kernal_size, self.kernal_size))
        erosion = cv2.erode(blurred, kernel, iterations = self.erosion_itterations)
        dilation = cv2.dilate(erosion, kernel, iterations = self.dilation_itterations)

        (dummy, blackAndWhiteImage) = cv2.threshold(dilation, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Defining points of a line to be drawn for visualizing error
        cam_center_line_x = int(self.image_width * self.camera_centerline)
        start_point = (cam_center_line_x,0)
        end_point = (cam_center_line_x, int(self.bottom_height))
        
        start_point_thresh_pos_x = int(cam_center_line_x -  (self.error_threshold * self.image_width/2))
        start_point_thresh_neg_x = int(cam_center_line_x + (self.error_threshold * self.image_width/2))
        
        start_point_thresh_pos = (start_point_thresh_pos_x, 0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(self.bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x, 0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(self.bottom_height))

        # Setting up data arrays
        cx_list = []
        cy_list = []

        # plotting contours and their centroids
        for contour in contours[:self.number_of_lines]:
            (cx,cy), r = cv2.minEnclosingCircle(contour)
            cx = int(cx)
            cy = int(cy)
            r = int(r)
            if self.min_width < r*2 < self.max_width:
                try:
                    img = cv2.circle(img, (cx, cy), r, (0,255,0),3)
                    
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        # Further image processing to determine optimal steering value
        try:
            # When only 1 road mark was found 
            if len(cx_list) == 1:
                self.get_logger().info("found a thing")
                mid_x, mid_y = cx_list[0], cy_list[0]
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)

                self.ek = float((mid_x - cam_center_line_x) / cam_center_line_x)
                lidar_center = int(2*self.n*(1 - mid_x / self.image_width))
                self.get_logger().info(f"lidar center: {lidar_center}")
                self.get_logger().info(f"{len(self.ranges)}")
                distance = self.ranges[lidar_center]

                self.get_logger().info(f"Found a guy: [tracking error: {self.ek}], [pixel radius: {r}], [distance prediction (in): {distance}]")

                # publish error data
                self.centroid_error.data = [float(self.ek), distance]
                self.centroid_error_publisher.publish(self.centroid_error)

            # When Nothing was found
            else:
                self.get_logger().info(f"Nothing detected")

            # clean slate
            error_list = [0] * self.number_of_lines
            cx_list = []
            cy_list = []
        except ValueError:
            pass

        # plotting results
        self.debug_cv = self.get_parameter('debug_cv').value # ability to update debug in real-time
        if self.debug_cv:
            cv2.imshow('img', img)
            cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()
    
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
