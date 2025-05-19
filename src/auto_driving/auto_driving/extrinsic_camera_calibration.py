import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.timer = self.create_timer(1.0, self.my_callback)

    def my_callback(self):
        MAX_ITERATIONS = 10
        for i in range(MAX_ITERATIONS):
            self.get_logger().info('Iteration %d' % i)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()