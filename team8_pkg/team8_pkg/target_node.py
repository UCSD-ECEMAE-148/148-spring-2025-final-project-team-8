import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64

import requests
import os

NODE_NAME = "target_node"
PROMPT_TOPIC = "/prompts"
API_KEY = os.getenv("GOOGLE_API_KEY")
MODEL_NAME = 'gemini-2.0-flash'
ENDPOINT = f'https://generativelanguage.googleapis.com/v1beta/models/{MODEL_NAME}:generateContent?key={API_KEY}'


class Target(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.prompt_listener = self.create_subscription(String, PROMPT_TOPIC, self.handle_prompt, 10)
  
  def handle_prompt(self, data):

    
    # Load image and convert to base64
    with open('your_image.jpg', 'rb') as img_file:
        image_bytes = img_file.read()
        image_base64 = base64.b64encode(image_bytes).decode('utf-8')

    # Construct the request body
    body = {
        "contents": [
            {
                "parts": [
                    {
                        "inline_data": {
                            "mime_type": "image/jpeg",
                            "data": image_base64
                        }
                    },
                    {
                        "text": data.data
                    }
                ]
            }
        ]
    }
    # Send the POST request
    response = requests.post(ENDPOINT, json=body)

    # Parse and print the result
    if response.ok:
        result = response.json()
        self.get_logger().info(result["candidates"][0]["content"]["parts"][0]["text"])
    else:
        self.get_logger().info(f"Error: {response.status_code}")
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
