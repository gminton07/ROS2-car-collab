import rclpy
from rclpy.node inport Node

from std_msgs.msg Float32, Bool

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Set up sensors

        self.subscription_angle = self.create_subscription(
                Float32,
                'lane_steering_angle',
                self.listener_angle,
                10)
        self.subscription_intersection = self.create_subscription(
                Bool,
                'intersection_detected',
                self.listener_intersection,
                10)
        self.subscription_red = self.create_subscription(
                Bool,
                'topic_red_detected',
                self.listener_red,
                10)
        self.subscription()

        self.get_logger().info('Initialized LED node.')

    def listener_angle():
        pass

    def listener_intersection():
        pass

    def listener_red():
        pass

    # LED Control functions


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if '__name__' == __main__:
    main()
