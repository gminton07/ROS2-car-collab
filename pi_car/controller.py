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
        [mx, my, mz, temp] = str(msg.data).split()
        mx = float(mx)
        my = float(my)
        mz = float(mz)
        temp = float(temp)
        if (self.status == 1 or self.status == 6):
            #print('mmu5603: ', data.data)
            #print(f'mx: {mx:.2f}\tmy: {my:.2f}\tmz: {mz:.2f}\ttemp: {temp:.1f}')
            self.get_logger().info(f'mx: {mx:.2f}\tmy: {my:.2f}\tmz: {mz:.2f}\ttemp: {temp:.2f}')

    def listener_imu(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + 'imu %s', data.data)
        
        [ax, ay, az, gx, gy, gz] = str(msg.data).split()
        ax = float(ax)  # destringify
        ay = float(ay)
        az = float(az)
        gx = float(gx)
        gy = float(gy)
        gz = float(gz)
        if (self.status == 2 or self.status == 6):
            #print(f'ax: {ax:.2f}\tmy: {ay:.2f}\taz: {az:.2f}')
            self.get_logger().info(f'ax: {ax:.2f}\tay: {ay:.2f}\taz: {az:.2f}\tgx: {gx:.2f}\tgy: {gy:.2f}\t gz: {gy:.2f}')

    def listener_ultra(self, msg):
        [distance] = str(msg.data).split()
        distance = float(distance)
        if (self.status == 3 or self.status == 6):
            #print(f'distance: {distance}')
            self.get_logger().info(f'distance: {distance:.2f}')

    def listener_camera(self, msg):
        frame =self.bridge.imgmsg_to_cv2(msg)
        #frame = ast.literal_eval(str(frame))
        if (self.status == 4 or self.status == 6):
            self.get_logger().info('Image received successfully!')

    def listener_mcp3008(self, msg):
        [adc, mvg_avg] = str(msg.data).split()
        adc = int(adc)
        mvg_avg = float(mvg_avg)
        if (self.status == 5 or self.status == 6):
            self.get_logger().info(f'adc: {adc}\tmvgAvg: {mvg_avg}')

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
