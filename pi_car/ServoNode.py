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
from std_msgs.msg import Float32, Bool, String

import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo 

import time
import adafruit_mmc56x3
import os

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Set up I2C and sensor
        self.i2c = board.I2C()
        self.sensor = adafruit_mmc56x3.MMC5603(self.i2c)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 60

        self.servo = servo.Servo(self.pca.channels[2])  # steering servo channel

        # Default (if no config file found)
        self.angle_center = 90.0
        self.angle_left = 60.0
        self.angle_right = 120.0

        self.load_servo_config()

        # Initialize servo position to center
        self.current_angle = self.angle_center
        self.servo.angle = self.current_angle

        # Subscriptions
        self.subscription_angle = self.create_subscription(
            Float32,
            'lane_steering_angle',
            self.listener_angle,
            10)
        
        self.subscription_intersection = self.create_subscription(
            Bool,
            'intersection_detected',
            self.listener_intersection,
            10)
        
        self.subscription_camera = self.create_subscription(
            Bool,
            'topic_red_detected',
            self.listener_camera,
            10)
        
        self.subscription = self.create_subscription(
            String,
            'topic_keyboard',
            self.listener_keyboard,
            10)

        self.get_logger().info('Initialized the subscriber node.')

        self.turning = False  # For managing intersection turns

    def load_servo_config(self):
        """Load servo angle settings from config file if available."""
        config_file = 'PICAR_CONFIG.txt'

        if os.path.exists(config_file):
            self.get_logger().info('Servo config file found. Loading values...')
            try:
                with open(config_file, 'r') as f:
                    lines = f.readlines()
                    if len(lines) >= 3:
                        self.angle_center = float(lines[7].strip())
                        self.angle_left = float(lines[6].strip())
                        self.angle_right = float(lines[8].strip())
                        self.get_logger().info(f'Loaded config: center={self.angle_center}, left={self.angle_left}, right={self.angle_right}')
                    else:
                        self.get_logger().warn('Config file found but not enough values.')
            except Exception as e:
                self.get_logger().error(f'Failed to load config: {e}')
        else:
            self.get_logger().info('No servo config found. Using defaults.')

    def map_steering_angle(self, normalized_angle):
        """Map a 0-180 normalized line angle into real calibrated angles."""
        # Assuming normalized_angle 90 should correspond to angle_center
        if normalized_angle < 90.0:
            # Left side
            mapped_angle = self.angle_center - (90.0 - normalized_angle) / 90.0 * (self.angle_center - self.angle_left)
        else:
            # Right side
            mapped_angle = self.angle_center + (normalized_angle - 90.0) / 90.0 * (self.angle_right - self.angle_center)
        
        # Clamp to avoid bad values
        mapped_angle = max(self.angle_left, min(self.angle_right, mapped_angle))
        return mapped_angle

    def listener_angle(self, msg):
        if not self.turning:
            line_angle = msg.data
            self.get_logger().info(f"Received lane steering angle: {line_angle}")

            mapped_angle = self.map_steering_angle(line_angle)
            self.get_logger().info(f"Mapped to servo angle: {mapped_angle}")

            self.servo.angle = mapped_angle
            self.current_angle = mapped_angle

    def listener_intersection(self, msg):
        if msg.data and not self.turning:
            self.get_logger().info("Intersection detected. Performing turn.")
            self.turning = True

            # Temporary turn (example: slight left or right)
            turn_angle = self.current_angle + 20 if self.current_angle < (self.angle_right - 20) else self.current_angle - 20
            self.servo.angle = turn_angle
            time.sleep(7)

            # Return to center
            self.servo.angle = self.current_angle
            time.sleep(0.2)

            self.turning = False

    def listener_camera(self, msg):
        if msg.data and not self.turning:
            self.get_logger().info("Camera triggered evasive turn.")
            self.turning = True

            # Example small evasive turn
            turn_angle = self.current_angle + 20 if self.current_angle < (self.angle_right - 20) else self.current_angle - 20
            self.servo.angle = turn_angle
            time.sleep(7)

            self.servo.angle = self.current_angle
            time.sleep(0.2)

            self.turning = False

            
    def listener_keyboard(self, msg):
        self.get_logger().info('Keyboard input: %s' % msg.data)
    
    # Define a small increment for slight steering
        slight_turn_amount = 5  # Adjust this value for how slight you want the turn to be

    # Check the keyboard input and steer accordingly
        if msg.data == 's':  # 's' for slight left turn
        # Slightly decrease the angle to turn left
            self.servo.angle = max(self.angle_left, self.current_angle - slight_turn_amount)
            self.get_logger().info(f"Slightly steering left: {self.servo.angle}")
        
        elif msg.data == 'd':  # 'd' for slight right turn
        # Slightly increase the angle to turn right
            self.servo.angle = min(self.angle_right, self.current_angle + slight_turn_amount)
            self.get_logger().info(f"Slightly steering right: {self.servo.angle}")
        
        elif msg.data == 'e':  # 'e' for straight (center)
            self.servo.angle = self.angle_center
            self.get_logger().info(f"Centering: {self.angle_center}")


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
