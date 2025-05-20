import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import depthai as dai
import numpy as np

NODE_NAME = "depth_node"
DEPTH_TOPIC = "/disparities"
FREQUENCY = 0.05
DISPLAY_DEPTH = True

class Depth(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.depth_publisher = self.create_publisher(Image, DEPTH_TOPIC, 10)
    self.bridge = CvBridge()

    self.create_pipeline()
    self.timer = self.create_timer(0.05, self.publish_depth)
    if DISPLAY_DEPTH:
      self.depth_viewer = self.create_subscription(Image, DEPTH_TOPIC, self.display_depth, 10)  
  
  def create_pipeline(self):
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    depth = pipeline.create(dai.node.StereoDepth)
    xout = pipeline.create(dai.node.XLinkOut)

    xout.setStreamName("disparity")

    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    depth.setLeftRightCheck(True)
    depth.setExtendedDisparity(False)
    depth.setSubpixel(False)

    # Linking
    monoLeft.out.link(depth.left)
    monoRight.out.link(depth.right)
    depth.disparity.link(xout.input)

    # Connect to device and start pipeline
    self.device = dai.Device(pipeline)
    # Output queue will be used to get the disparity frames from the outputs defined above
    self.queue = self.device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
    self.maxDisparity = depth.initialConfig.getMaxDisparity()

  def publish_depth(self):
    inDisparity = self.queue.get()
    frame = inDisparity.getFrame()
    frame = (frame * (255 / self.maxDisparity)).astype(np.uint8)
    image = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
    self.depth_publisher.publish(image)

  def display_depth(self, data):
    frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
    cv2.imshow("disparity_color", frame)
    cv2.waitKey(1)

def main():
  rclpy.init()
  depth_node = Depth()
  try:
    rclpy.spin(depth_node)
    depth_node.destroy_node()
    rclpy.shutdown()
  except KeyboardInterrupt:
    depth_node.get_logger().info(f'Shutting down {NODE_NAME}...')

    # Kill cv2 windows and node
    depth_node.destroy_node()
    rclpy.shutdown()
    depth_node.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == "__main__":
  main()
