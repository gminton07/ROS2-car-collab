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
from sensor_msgs.msg import Image

from cv_bridge import CvBridge  # For converting camera msg back into image

import time

class SubscriberPublisher(Node):
    
    # Initialize Global variables
    mx, my, mz, temp = (None, None, None, None)
    ax, ay, az, gx, gy, gz = (None, None, None, None, None, None)
    distance = None
    frame = None
    adc, adc_diff, mvg_avg = (None, None, None)


    def __init__(self):
        super().__init__('subscriber_publisher')
        self.subscription = self.create_subscription(
            String,
            'topic_mmc5603',
            self.listener_mmu5603,
            10)
        self.subscription = self.create_subscription(
            String,
            'topic_imu',
            self.listener_imu,
            10)
        self.subscription = self.create_subscription(
            String,
            'topic_ultra',
            self.listener_ultra,
            10)
        self.subscription = self.create_subscription(
            Image,
            'topic_camera',
            self.listener_camera,
            10)
        self.subscription = self.create_subscription(
            String, 
            'topic_mcp3008',
            self.listener_mcp3008,
            10)
        self.subscription = self.create_subscription(
            String,
            'topic_keyboard',
            self.listener_keyboard,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')
        self.publisher_motor = self.create_publisher(String, 'topic_motor', 10)
        self.publisher_servo = self.create_publisher(String, 'topic_servo', 10)
        # status is a global that I'll use to print
        # 1-5603    2-imu   3-ultra 4-camera    5-3008  6-all
        # While I'm not a fan of globals, in this case, you can use 
        # globals to pass information from one of the call functions to the other
        self.status = 6
        self.bridge = CvBridge()


    def listener_mmu5603(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + 'mmc5603 %s', data.data)
        [self.mx, self.my, self.mz, self.temp] = str(msg.data).split()
        self.mx = float(self.mx)
        self.my = float(self.my)
        self.mz = float(self.mz)
        self.temp = float(self.temp)
        if (self.status == 1 or self.status == 6):
            #print('mmu5603: ', data.data)
            #print(f'self.mx: {self.mx:.2f}\tself.my: {self.my:.2f}\tself.mz: {self.mz:.2f}\tself.temp: {self.temp:.1f}')
            self.get_logger().info(f'self.mx: {self.mx:.2f}\tself.my: {self.my:.2f}\tself.mz: {self.mz:.2f}\tself.temp: {self.temp:.2f}')

    def listener_imu(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + 'imu %s', data.data)
        
        [self.ax, self.ay, self.az, self.gx, self.gy, self.gz] = str(msg.data).split()
        self.ax = float(self.ax)  # destringify
        self.ay = float(self.ay)
        self.az = float(self.az)
        self.gx = float(self.gx)
        self.gy = float(self.gy)
        self.gz = float(self.gz)
        if (self.status == 2 or self.status == 6):
            #print(f'self.ax: {self.ax:.2f}\tself.my: {self.ay:.2f}\tself.az: {self.az:.2f}')
            self.get_logger().info(f'self.ax: {self.ax:.2f}\tself.ay: {self.ay:.2f}\tself.az: {self.az:.2f}\tself.gx: {self.gx:.2f}\tself.gy: {self.gy:.2f}\t self.gz: {self.gz:.2f}')

    def listener_ultra(self, msg):
        [self.distance] = str(msg.data).split()
        self.distance = float(self.distance)
        if self.distance < 50:
            self.publish_motor(dc=0, direction=0)
        if (self.status == 3 or self.status == 6):
            #print(f'self.distance: {self.distance}')
            self.get_logger().info(f'self.distance: {self.distance:.2f}')

    def listener_camera(self, msg):
        self.frame =self.bridge.imgmsg_to_cv2(msg)
        if (self.status == 4 or self.status == 6):
            self.get_logger().info('Image received successfully!')

    def listener_mcp3008(self, msg):
        [self.adc, self.adc_diff, self.mvg_acg] = str(msg.data).split()
        self.adc = int(self.adc)
        self.adc_diff = int(self.adc_diff)
        self.mvg_acg = float(self.mvg_acg)
        if (self.status == 5 or self.status == 6):
            self.get_logger().info(f'self.adc: {self.adc}\tself.adc_diff: {self.adc_diff}\tmvgAvg: {self.mvg_acg}')

    def listener_keyboard(self, msg):
        self.get_logger().info('Keyboard %s' % msg.data)
        #print("keyboard: " + str(data.data))

        if (isinstance(int(str(msg.data)), int)):
            self.status = int(str(msg.data))
            if (self.status > 4):
                self.status = 0

    def publish_motor(self, dc, direction):
        # Expected Values:
        # direction: -1 backward, 0 stop, 1 forward
        # Duty cycle: percentage
        msg = String()
        dutyCycle = dc
        direction = direction
        msg.data = "{0} {1}".format(dutyCycle, direction)   # TODO: change msg format
        self.publisher_motor.publish(msg)
        self.get_logger().info('Publishing motor: "%s"' % msg.data)

    def publish_servo(self, args):
        # Assumes args holds a list of PWM values for respective servos
        # Should accept values on range [-10, 10] for left -> right
        msg = String()
        [nod, swivel, steer] = args
        msg.data = "{0} {1} {2}".format(nod, swivel, steer)
        self.publisher_servo.publish(msg)
        self.get_logger().info('Publishing servo: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    subscriber_publisher = SubscriberPublisher()

    subscriber_publisher.publish_motor(dc=50, direction=1)
    subscriber_publisher.publish_servo(args=[0, 0, 0])
    time.sleep(5)
    subscriber_publisher.publish_motor(dc=70, direction=-1)
    subscriber_publisher.publish_servo(args=[10, 10, 10])
    
    #TODO: This is one method for updating motor and servo values
    # there's probably something better. If these 3 function calls (motor to spin)
    # are in a (while True:) block that could work.
    

    rclpy.spin(subscriber_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
