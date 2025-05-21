
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

        Kp = 0.0032
        Kd = 0.0006

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        max_speed = 0.1
        twist = Twist()
        twist.linear.x = min(30 * (max(1 - abs(error) / 500, 0) ** 2.2), max_speed)

        limit = 3.0
        twist.angular.z = -max(angular_z, -limit) if angular_z < 0 else -min(angular_z, limit)

        # 속도와 조향 연동 (속도 빠를수록 조향 감도 줄임)
        twist.angular.z *= max(1 - twist.linear.x / max_speed, 0.5)

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
