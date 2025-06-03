import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LIDAR_TOPIC = "/scan"

class Target(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.prompt_listener = self.create_subscription(String, PROMPT_TOPIC, self.handle_prompt, 10)
  
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
