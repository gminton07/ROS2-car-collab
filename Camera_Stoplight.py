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


        #FIND RED
        frame = cv_image
            # ANALYZE 
        #cv2.imwrite('hsv.jpg', frame)
        hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # mask&ranges
        low = (0, 100, 100)
        high = (179, 255, 255)
        mask=cv2.inRange(hsv_img, low, high)
        #cv2.imwrite('mask1.jpg', mask)
        count = np.sum(np.nonzero(mask))
        #print("count =",count)
	upperbound=10000000
        if count<upperbound:
            result.data = False
        else:
            result.data = True
    
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
