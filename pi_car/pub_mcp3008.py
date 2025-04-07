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

# Import MCP3008 libraries
import Adafruit_MCP3008
import RPi.GPIO as GPIO
from Adafruit_GPIO.GPIO import RPiGPIOAdapter as Adafruit_GPIO_Adapter

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic_mcp3008', 10)
        timer_period = 2  # TODO: change this later
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # Configure SPI
        self.CLK = 40   # pin 13
        self.MISO = 21  # pin 12
        self.MOSI = 19  # pin 11
        self.CS = 26    # pin 10
        self.gpio_adapter = Adafruit_GPIO_Adapter(GPIO, mode=GPIO.BOARD)
        self.sensor = Adafruit_MCP3008.MCP3008(clk=self.CLK, cs=self.CS, miso=self.MISO, mosi=self.MOSI, gpio=self.gpio_adapter)

    def timer_callback(self):
        msg = String()
        adc_read = self.sensor.read_adc(0) # TODO: Verify which pin is used
        msg.data = "{0:10f}".format(adc_read)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
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
