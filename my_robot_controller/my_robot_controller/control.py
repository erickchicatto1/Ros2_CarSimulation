import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32

from ackermann_msgs.msg import AckermannDriveStamped


class PIDController(Node):

    def __init__(self):

        super().__init__('pid_controller')

        self.subscription = self.create_subscription(
            Float32,
            '/error',
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.prev_steering_error = 0.0

        self.vel_input = 2.0

        self.s_kp = 1.5
        self.s_kd = 0.2

    def callback(self, msg):

        steering_error = msg.data

        steering_error_diff = (
            steering_error -
            self.prev_steering_error
        )

        self.prev_steering_error = steering_error

        angle = (
            self.s_kp * steering_error +
            self.s_kd * steering_error_diff
        )

        angle = max(min(angle, 0.4), -0.4)

        drive_msg = AckermannDriveStamped()

        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = self.vel_input

        self.publisher_.publish(drive_msg)

        self.get_logger().info(
            f"angle={angle:.3f} speed={self.vel_input}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PIDController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
