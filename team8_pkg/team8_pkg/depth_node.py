import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces.msg import CameraData
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
    self.depth_publisher = self.create_publisher(CameraData, DEPTH_TOPIC, 10)
    self.bridge = CvBridge()

    self.create_pipeline()
    self.timer = self.create_timer(0.05, self.publish_depth)
    self.data = CameraData()
    if DISPLAY_DEPTH:
      self.data_viewer = self.create_subscription(CameraData, DEPTH_TOPIC, self.display_data, 10)  
  
  def create_pipeline(self):
    pipeline = dai.Pipeline()

    #color camera pipeline
    colorCam = pipeline.create(dai.node.ColorCamera)
    colorCam.setInterleaved(False)
    colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    depth = pipeline.create(dai.node.StereoDepth)
    xout_depth = pipeline.create(dai.node.XLinkOut)

    xout_depth.setStreamName("disparity")

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
    depth.disparity.link(xout_depth.input)
    colorCam.preview.link(xout_rgb.input)

    # Connect to device and start pipeline
    self.device = dai.Device(pipeline)
    # Output queue will be used to get the disparity frames from the outputs defined above
    self.queue_disp = self.device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
    self.queue_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    self.maxDisparity = depth.initialConfig.getMaxDisparity()

  def publish_depth(self):
    #depth
    inDisparity = self.queue_disp.get()
    disp = inDisparity.getFrame()
    disp = (disp * (255 / self.maxDisparity)).astype(np.uint8)
    self.data.disparity = self.bridge.cv2_to_imgmsg(disp, encoding="mono8")

    # RGB
    inRgb = self.queue_rgb.get()
    rgb_frame = inRgb.getCvFrame()
    self.data.rgb = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")

    self.depth_publisher.publish(self.data)

  def display_data(self, data):
    frame = self.bridge.imgmsg_to_cv2(data.disparity, desired_encoding="passthrough")
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
