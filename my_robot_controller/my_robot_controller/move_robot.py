import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class WallDistanceFinder(Node):

    def __init__(self):
        super().__init__('wall_distance_finder')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Float32,
            '/error',
            10
        )

        self.wall_projection_r = 350

        self.target_distance = 0.7

        self.error = 0.0

    def callback(self, msg):

        wall_distance_r = (
            0.5 * msg.ranges[self.wall_projection_r]
        )

        steering_error = (
            self.target_distance -
            wall_distance_r
        )

        error_msg = Float32()

        error_msg.data = float(steering_error)

        self.publisher_.publish(error_msg)

        self.get_logger().info(
            f"Publishing Steering Error: {error_msg.data}"
        )


def main(args=None):

    rclpy.init(args=args)

    wall_distance_finder = WallDistanceFinder()

    rclpy.spin(wall_distance_finder)

    wall_distance_finder.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
