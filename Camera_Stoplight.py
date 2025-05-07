import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class MinimalPublisher(Node):

    def __init__(self):
        #set it to be in minimul publisher 
        super().__init__('minimal_publisher')

        self.bridge = CvBridge() 
        #setup subscriber to camera 
        self.subscription = self.create_subscription(
            Image,
            'topic_camera',
            self.image_callback,
            10)

        #setup publisher of if red light is detected 
        self.publisher_ = self.create_publisher(Bool, 'topic_red_detected', 10)
        # sampling period
        timer_period = 1  # seconds
        self.i = 0
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")
            return
        
        #change to boolean 
        result = Bool()
        result.data = False

        # red detection within top half of photo 
        frame = cv_image
        # dimensions top half
        height, width = frame.shape[:2]
        top_half = frame[0:int(height / 2), :]

        # convert to HSV
        hsv_img = cv2.cvtColor(top_half, cv2.COLOR_BGR2HSV)

        # red color range in HSV (adjust if needed)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # masks for red detection (two ranges for red hue wraparound)
        mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # count non-zero (red) pixels 
        count = np.sum(mask > 0)
        prting(count)
        # threshold of non-zero pixels to count as stoplight 
        upperbound = 100000
        if count > upperbound:
            result.data = True
        else:
            result.data = False

    
        self.publisher_.publish(result)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
