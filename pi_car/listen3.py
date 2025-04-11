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

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
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
            'topic_keyboard',
            self.listener_keyboard,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')
        # status is a global that I'll use to print
        # 0 - imu   1 - 5603    2 - both    3 - neither
        # While I'm not a fan of globals, in this case, you can use 
        # globals to pass information from one of the call functions to the other
        self.status = 0 


    def listener_mmu5603(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + 'mmc5603 %s', data.data)
        [mx, my, mz] = str(msg.data).split()
        mx = float(mx)
        my = float(my)
        mz = float(mz)
        if (self.status == 1 or self.status == 2):
            #print('mms5603: ', data.data)
            #print(f'mx: {mx:.2f}\tmy: {my:.2f}\tmz: {mz:.2f}\ttemp: {temp:.1f}')
            self.get_logger().info(f'mx: {mx:.2f}\tmy: {my:.2f}\tmz: {mz:.2f}')

    def listener_imu(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + 'imu %s', data.data)
        [ax, ay, az] = str(msg.data).split()
        ax = float(ax)  # destringify
        ay = float(ay)
        az = float(az)
        if (self.status == 0 or self.status == 2):
            #print(f'ax: {ax:.2f}\tmy: {ay:.2f}\taz: {az:.2f}')
            self.get_logger().info(f'ax: {ax:.2f}\tmy: {ay:.2f}\taz: {az:.2f}')

    def listener_keyboard(self, msg):
        self.get_logger().info('Keyboard %s' % msg.data)
        #print("keyboard: " + str(data.data))

        if (isinstance(int(str(msg.data)), int)):
            self.status = int(str(msg.data))
            if (self.status > 4):
                self.status = 0

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
