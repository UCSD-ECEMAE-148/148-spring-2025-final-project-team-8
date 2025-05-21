import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from google import genai

NODE_NAME = "target_node"
PROMPT_TOPIC = "/prompts"

client = genai.Client(api_key="AIzaSyAKcwfdHu1iRh4ZWRVyaT55rd39OypqyrU")

class Target(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.prompt_listener = self.create_subscription(String, PROMPT_TOPIC, self.handle_prompt, 10)
  
  def handle_prompt(self, data):
    response = client.models.generate_content(
    model="gemini-2.0-flash",
    contents=data.data,
    )
    self.get_logger().info(response.text)
  
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
