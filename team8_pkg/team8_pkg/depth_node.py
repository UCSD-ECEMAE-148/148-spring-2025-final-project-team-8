import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import depthai as dai
import numpy as np

NODE_NAME = "depth_node"
DEPTH_TOPIC = "/depths"

class Depth(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.depth_publisher = self.create_publisher(

    self.create_pipeline()
  
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
    self.queue = device.getOutputQueue(name="disparity", maxSize=4, blocking=True)
    self.maxDisparity = depth.initialConfig.getMaxDisparity()

        while True:
            inDisparity = q.get()  # blocking call, will wait until a new data has arrived
            self.get_logger().info("Frame received!")
            frame = inDisparity.getFrame()
            frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            # Normalization for better visualization
            frame = (frame * (255 / maxDisparity)).astype(np.uint8)

            #uncomment to show mono
            #cv2.imshow("disparity", frame)

            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            #frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            #cv2.imshow("disparity_color", frame)
            #self.get_logger().info("Frame rendered!")

            if cv2.waitKey(1) == ord('q'):
                break


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
