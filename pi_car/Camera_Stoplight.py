import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import cv2
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        #set it to be in minimul publisher 
        super().__init__('minimal_publisher')
        #setup publisher 
        self.publisher_ = self.create_publisher(Bool, 'topic_camera', 10)
        # sampling period
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        

    def timer_callback(self):
        #change to boolean 
        msg = Bool()
        msg.data = False 

        #FIND RED
            # open camera
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
            # set dimensions
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
            # take frame
        ret, frame = cap.read()
            # release camera
        cap.release()
            # ANALYZE 
        cv2.imwrite('hsv.jpg', frame)
        hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # mask&ranges
        low = (0, 100, 100)
        high = (179, 255, 255)
        mask=cv2.inRange(hsv_img, low, high)
        cv2.imwrite('mask1.jpg', mask)
        count = np.sum(np.nonzero(mask))
        #print("count =",count)
        if count == 0:
            #print("Not Red")
            msg.data = False; 
        else:
            #print("Red")
            msg.data = True;      
        self.publisher_.publish(msg)
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
