import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import Twist
import time
import os

from enum import Enum

NODE_NAME = 'lane_guidance_node'
CENTROID_TOPIC_NAME = '/location'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
CONTROL_INPUT_TOPIC_NAME = '/cin'
CONTROL_OUTPUT_TOPIC_NAME = '/cout'
THRESHOLD_DIST = 2.07
THRESHOLD_VAR = 0.03
THRESHOLD_VAR_FINE = 0.2
LOCKON = 2
BIAS = 0.04

class DistanceState(Enum):
    INRANGE = 1
    FAR = 2
    NEAR = 3
class AngleState(Enum):
    CENTER = 1
    RIGHT = 2
    LEFT = 3

class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.centroid_subscriber = self.create_subscription(Float32MultiArray, CENTROID_TOPIC_NAME, self.controller, 10)
        self.centroid_subscriber
        self.control_publisher = self.create_publisher(String, CONTROL_INPUT_TOPIC_NAME, 10)
        self.control_subscriber = self.create_subscription(String, CONTROL_OUTPUT_TOPIC_NAME, self.on_control, 10)
        self.control_subscriber

        self.lockon = False
        self.begin_lock = -1
        self.launch = False

        self.fine = False

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0)
            ])
        self.Kp = self.get_parameter('Kp_steering').value # between [0,1]
        self.Ki = self.get_parameter('Ki_steering').value # between [0,1]
        self.Kd = self.get_parameter('Kd_steering').value # between [0,1]
        self.error_threshold = self.get_parameter('error_threshold').value # between [0,1]
        self.zero_throttle = self.get_parameter('zero_throttle').value # between [-1,1] but should be around 0
        self.max_throttle = self.get_parameter('max_throttle').value # between [-1,1]
        self.min_throttle = self.get_parameter('min_throttle').value # between [-1,1]
        self.max_right_steering = self.get_parameter('max_right_steering').value # between [-1,1]
        self.max_left_steering = self.get_parameter('max_left_steering').value # between [-1,1]

        # initializing PID control
        self.Ts = float(1/20)
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8
        self.fine_counter = 0
        
        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )
    def on_control(self, data):
        if data.data == "launch":
            self.get_logger().info("LAUNCHING")
            self.launch = True

    def controller(self, data):
        if not self.launch:
            self.get_logger().info("Waiting for Launch...")
            #self.get_logger().info(f"Distance: {data.data[1]}")
            return
        #ignore input if locked on
        if self.lockon:
            self.lock_target()
            self.lockon = False
            return
        
        # setting up PID control
        self.ek = data.data[0]
        if len(data.data) > 1:
            self.distance = data.data[1]

        # Throttle gain scheduling (function of error)
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Steering PID terms
        self.proportional_error = self.Kp * self.ek
        self.derivative_error = self.Kd * (self.ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * self.ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

        # check state
        distance_state = 0
        angle_state = 0
        if self.ek < BIAS + self.error_threshold and self.ek > BIAS - self.error_threshold:
            self.get_logger().info("Dead ahead!")
            angle_state = AngleState.CENTER
        else:
            if self.ek > 0:
                self.get_logger().info("Left!")
                angle_state = AngleState.LEFT
            else:
                self.get_logger().info("Right!")
                angle_state = AngleState.RIGHT
        
        if self.distance > THRESHOLD_DIST - THRESHOLD_VAR:
            if self.distance < THRESHOLD_DIST + THRESHOLD_VAR:
                self.get_logger().info("In range!")
                distance_state = DistanceState.INRANGE
            else:
                self.get_logger().info("Too far!")
                distance_state = DistanceState.FAR
        else:
            self.get_logger().info("Too close!")
            distance_state = DistanceState.NEAR
        
        # enter fine control
        if angle_state == AngleState.CENTER and self.distance > THRESHOLD_DIST - THRESHOLD_VAR_FINE and self.distance < THRESHOLD_DIST + THRESHOLD_VAR_FINE:
            self.fine_counter += 1
        else:
            self.fine_counter = 0
            self.fine = False
        
        if self.fine_counter >= 5:
            self.fine = True
            self.get_logger().info("FINE CONTROL")

        #handle state
        # in range
        if distance_state == DistanceState.INRANGE and angle_state == AngleState.CENTER:
            self.get_logger().info("LOCKING")
            if self.begin_lock < 0:
                self.begin_lock = time.time()
            else:
                if time.time() - self.begin_lock >= LOCKON:
                    self.lockon = True
        else:
            self.begin_lock = -1
        
        if distance_state == DistanceState.INRANGE: 
            if angle_state == AngleState.CENTER:
                # if centered, do nothing
                throttle_float = self.zero_throttle
            else:                               
                # if not centered, reverse to face the centroid
                throttle_float = -throttle_float
                steering_float = -steering_float
        # too cloase
        if distance_state == DistanceState.NEAR:
            # reverse to face
            throttle_float = -throttle_float
            steering_float = -steering_float
        #if distance_state == DistanceState.FAR:
            # work as usual
        

        # Publish values
        try:
            # publish control signals
            self.twist_cmd.angular.z = float(steering_float)
            self.twist_cmd.linear.x = float(throttle_float)
            self.twist_publisher.publish(self.twist_cmd)
            
            if self.fine:
                time.sleep(0.1)
                self.twist_cmd.linear.x = self.zero_throttle
                self.twist_publisher.publish(self.twist_cmd)
                time.sleep(1.5)

            # shift current time and error values to previous values
            self.ek_1 = self.ek

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 

    def lock_target(self):
        self.get_logger().info("LOCKED ON")
        msg = String()
        msg.data = "shoot"
        self.control_publisher.publish(msg)
        time.sleep(100)
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
