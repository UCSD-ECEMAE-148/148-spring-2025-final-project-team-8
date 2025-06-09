import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50
servo = servo.Servo(pca.channels[0])

NODE_NAME = "control_node"
TOPIC_NAME = "/cin"
OUTPUT_TOPIC_NAME = "/cout"

class Control(Node):
  def __init__(self):
    super().__init__(NODE_NAME)
    self.control_subscriber = self.create_subscription(String, TOPIC_NAME, self.on_control, 10)
    self.control_publisher = self.create_publisher(String, OUTPUT_TOPIC_NAME, 10)
    self.trigger_state = True
  
  def on_control(self, data):
    if data.data == "shoot":
      self.trigger_state = True
    if data.data == "reload":
      self.trigger_state = False
    if data.data == "launch":
      msg = String()
      msg.data = "launch"
      self.control_publisher.publish(msg)
    
    self.handle_trigger_state()

  def handle_trigger_state(self):
    if self.trigger_state:
      servo.angle = 90
    else:
      servo.angle = 0
  
def main():
  rclpy.init()
  control_node = Control()
  try:
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()
  except KeyboardInterrupt:
    control_node.get_logger().info(f'Shutting down {NODE_NAME}...')

    # Kill cv2 windows and node
    control_node.destroy_node()
    rclpy.shutdown()
    control_node.get_logger().info(f'{NODE_NAME} shut down successfully.')

    pca.deinit()

if __name__ == "__main__":
  main()
