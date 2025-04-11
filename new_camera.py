import rclpy, cv2
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'topic_camera', 10)
        timer_period = 2 # seconds TODO: change later
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Initialize camera
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # TODO: 640 x 480 or 2560 x 1440?

        self.bridge = CvBridge()

    def timer_callback(self):

        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        else:
            self.cap.set(cv2.CAP_PROP_FRAMES, 0)

        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        '''
        if self.i > 2:
            cv2.imwrite(msg.data, 'image.jpg')
        '''


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy node
    self.cap.release()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
