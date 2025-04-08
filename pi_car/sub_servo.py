import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
                String,
                'topic_servo',
                self.listener_servo,
                10)
        self.subscription   # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')

    # TODO: Get servo updater code

    def listener_servo(self, msg):
        [steer, swivel, nod] = str(msg.data).split()     # TODO: Change to preferred variables
        steer = float(steer)
        swivel = float(swivel)
        nod = float(nod)

        self.get_logger().info(f'steer: {steer}\tswivel:{swivel}\tnod:{nod}')


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
