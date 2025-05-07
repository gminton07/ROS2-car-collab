# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'topic_camera', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # TODO: change to 640 x 480 or 2560 x 1440?

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.publisher_.publish(img_msg)
                #self.get_logger().info(f'Image {self.i} published successfully!')
            except CvBridgeError as error:
                print(error)
                self.get_logger().info(f'Image failed to send :(')
        self.i += 1
               

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    self.cap.release()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
