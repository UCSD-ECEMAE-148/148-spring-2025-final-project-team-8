import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

NODE_NAME = "target_node"
TOPIC_NAME = "/location"

class Target(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.centroid_publisher = self.create_publisher(Float32, TOPIC_NAME, 10)

    def callback(packet: DetectionPacket):
        for d in packet.img_detections.detections:
            (d.xmax - d.xmin) / 2 + d.xmin
            

    with OakCamera() as oak:
        color = oak.create_camera('color')
        model_config = {
            'source': 'roboflow', # Specify that we are downloading the model from Roboflow
            'model':'detect-faces-m1pbd/6',
            'key':'3omR0u9ATOe8U1rQmZwo'
        }
        nn = oak.create_nn(model_config, color)
        oak.visualize([nn], fps=True, callback=callback)
        oak.start(blocking=True)
  
  def handle_prompt(self, data):
    self.get_logger().info(data.data)
  
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
