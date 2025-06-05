import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

NODE_NAME = "lidar_node"
LIDAR_TOPIC = "/scan"

class Lidar(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.prompt_listener = self.create_subscription(LaserScan, LIDAR_TOPIC, self.handle_prompt, 10)
    self.ker_size = 6
  
  def handle_prompt(self, data):
    self.get_logger().info("--------------------")
    self.N = len(data.ranges)
    self.get_logger().info(f'{self.N} points')
    self.n = int((self.N / 360.0) * 35)
    for i in range(-self.n, self.n):
      if data.ranges[i] < 0.5:
        self.get_logger().info(f'Index {i} is close by: {data.ranges[i]}')
    
  
def main():
  rclpy.init()
  target_node = Lidar()
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
