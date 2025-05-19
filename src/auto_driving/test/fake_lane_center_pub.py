import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class FakeControlLaneInput(Node):
    def __init__(self):
        super().__init__('fake_control_lane_input')
        self.publisher_ = self.create_publisher(Float64, '/control/lane', 10)

        self.start_time = time.time()
        self.frequency = 0.2      # 주파수 (Hz)
        self.timer = self.create_timer(0.05, self.publish_sin_input) # 20Hz

    def publish_sin_input(self):
        t = time.time() - self.start_time
        value = 500 + 200 * math.sin(2 * math.pi * 0.2 * t)
        msg = Float64()
        msg.data = value
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeControlLaneInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
