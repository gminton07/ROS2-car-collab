import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
                String,
                'topic_motor',
                self.listener_motor,
                10)
        self.subscription   # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')

    # TODO: import all motor functions and update GPIO pins within listener_motor

    def listener_motor(self, msg):
        [dC, direction] = str(msg.data).split()     # TODO: Change to preferred variables
        dC = float(dC)

        self.get_logger().info(f'duty cycle: {dC}\tdirection: {direction}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy node explicitly
    # (optional - otherwise done automatically
    # when garbage collector comes
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
