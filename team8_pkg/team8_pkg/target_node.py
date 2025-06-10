import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LabeledCentroid
from depthai_sdk.classes import DetectionPacket
from depthai_sdk import OakCamera

NODE_NAME = "target_node"
TOPIC_NAME = "/centroids"

MODEL_NAME = "detect-buckets/1"

class Target(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.centroid_publisher = self.create_publisher(LabeledCentroid, TOPIC_NAME, 10)
    self.model_loaded = False

    def callback(packet: DetectionPacket):
        if not self.model_loaded:
            self.get_logger().info(f'Successfully loaded model: {MODEL_NAME}')
            self.model_loaded = True

        for d in packet.img_detections.detections:
            centroid = LabeledCentroid()
            centroid.label = d.label
            centroid.cx = (d.xmax - d.xmin) / 2 + d.xmin
            self.centroid_publisher.publish(centroid)


    with OakCamera() as oak:
        color = oak.create_camera('color')
        model_config = {
            'source': 'roboflow', # Specify that we are downloading the model from Roboflow
            'model': MODEL_NAME,
            'key':'3omR0u9ATOe8U1rQmZwo'
        }
        nn = oak.create_nn(model_config, color)
        oak.visualize([nn], fps=True, callback=callback)
        oak.start(blocking=True)
  
def main():
  rclpy.init()
  target_node = Target()
  try:
    rclpy.spin(target_node)
    target_node.destroy_node()
    rclpy.shutdown()
  except KeyboardInterrupt:
    target_node.get_logger().info(f'Shutting down {NODE_NAME}...')

    # Kill cv2 windows and node
    target_node.destroy_node()
    rclpy.shutdown()
    target_node.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == "__main__":
  main()
