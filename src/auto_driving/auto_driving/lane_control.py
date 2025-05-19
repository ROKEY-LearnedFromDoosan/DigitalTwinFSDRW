
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/detect/lane',
            self.callback_follow_lane,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )

        # PD control related variables
        self.last_error = 0

    def callback_follow_lane(self, desired_center):

        center = desired_center.data
        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        twist = Twist()
        # Linear velocity: adjust speed based on error (maximum 0.05 limit)
        twist.linear.x = min(50 * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)
        self.get_logger().info(f'Published to /cmd_vel: {twist.linear.x:.3f},{twist.angular.z:.3f}')

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
