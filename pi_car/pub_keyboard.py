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

from std_msgs.msg import String

class MinimalPublisher(Node):

    # modified from stackoverflow thread: 
    # https://stackoverflow.com/questions/3523174/raw-input-without-pressing-enter
    def getch(self):
        # usage, gets a single keystroke without the need to press return
        # c = getch()

        import sys, termios, tty

        fd = sys.stdin.fileno()
        orig = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)  # or tty.setraw(fd) if you prefer raw mode's behavior.
            msg = sys.stdin.read(1)
            self.get_logger().info('Publishing key %s ' % msg)
            #return sys.stdin.read(1)
            return msg
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, orig)

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic_keyboard', 10)
        keyin = String()

        while rclpy.ok():
            keyin.data = self.getch() 
            self.publisher_.publish(keyin)
    
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
