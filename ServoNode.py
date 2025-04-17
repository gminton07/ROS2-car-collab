#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo 

import time
#import argparse
import adafruit_mmc56x3
import os

import RPi.GPIO as GPIO

class ServoSub(Node):
    def __init__(self):
        super().__init__('servo_subscriber')

        # Set up I2C and sensor (just like other sensor codes)
        self.i2c = board.I2C()
        self.sensor = adafruit_mmc56x3.MMC5603(self.i2c)
        self.pca = PCA9685(self.i2c)

        # Set the PWM frequency to 60hz.
        self.pca.frequency = 60

        # Attaching servo to channel 0 
        self.servo = servo.Servo(self.pca.channels[0])

        # Default calibration angles
        self.angle_left = 60.0
        self.angle_center = 90.0
        self.angle_right = 120.0

        # Load calibration from file if available
        self.load_servo_config()

        # Set initial neutral angle
        self.current_angle = self.angle_center
        self.servo.angle = self.current_angle

        #create a subsription to get angles from line detector node 
        self.subscription = self.create_subscription(
            Float32,
            'lane_steering_angle',
            self.listener_angle,
            10)
        
        self.subscription = self.create_subscription(
            Bool,
            'intersection_detected',
            self.listener_angle,
            10)

        
        self.subscription = self.create_subscription(
            Bool,
            'topic_camera',
            self.listener_intersection,
            10)
        

        self.subscription  # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')
    
        self.status = False  # Camera-triggered turning status


        #Grab boolean from camera
        '''
        Either when the motor stops steer by a small angle and keep going 
        for a while and then straighten the steer(lots of trial and error to get
        idaal timing on that), but ussues with figuring wheter to turn left or right 
        and also IT WOULD MAKESENSE FOR MAGNETOMETER/ACCELEROMETER STUFF TO BE IN WITH THE SERVO
    '''
        
    def load_servo_config(self):
        """Load servo angle settings from a config file if it exists."""
        config_file = 'PICAR_CONFIG.txt'

        if os.path.exists(config_file):
            self.get_logger().info('Servo config file found. Loading values...')
            try:
                with open(config_file, 'r') as f:
                    lines = f.readlines()
                    if len(lines) >= 3:
                        self.angle_center = float(lines[0].strip())
                        self.angle_left = float(lines[1].strip())
                        self.angle_right = float(lines[2].strip())
                        self.get_logger().info(f'Config loaded: center={self.angle_center}, '
                                               f'left={self.angle_left}, right={self.angle_right}')
                    else:
                        self.get_logger().warn('Config file found but has fewer than 3 lines.')
            except Exception as e:
                self.get_logger().error(f'Failed to load config: {e}')
        else:
            self.get_logger().info('No servo config file found. Using default values.')

    def listener_angle(self, msg):
        if not self.turning:  # Only adjust if camera is not triggering turn
            angle = msg.data
            self.get_logger().info(f"Received line angle: {angle}")

        # Clamp or calibrate angle if necessary
            angle = max(0.0, min(180.0, angle))  # Adjust depending on servo range
            self.servo.angle = angle
            self.current_angle = angle

    def listener_intersection(self, msg):
        if not self.turning:
            if msg.data:  # Boolean is True → camera triggered event
                self.get_logger().info("Intersection detection triggered: Executing turn.")
                self.turning = True

        # Example behavior: slight turn for 7 seconds, then return to center
            self.servo.angle = self.current_angle + 20 if self.current_angle < 160 else self.current_angle - 20
            time.sleep(7)
            self.servo.angle = self.current_angle
            time.sleep(0.2)

            self.turning = False

        
        "..... Get angle from line detection and set servo angle to steer  that"
        '''
            Potential issues with angle is that it could end up in accidentally over correcting. 
        Make sure once you correct position that becomes the new center, figure something 
        out with callibration
        '''
    def listener_camera(self,msg):
        if msg.data:  # Boolean is True → camera triggered event
            self.get_logger().info("Camera triggered: Executing evasive turn.")
            self.turning = True

            # Example behavior: slight turn for 7 seconds, then return to center
            self.servo.angle = self.current_angle + 20 if self.current_angle < 160 else self.current_angle - 20
            time.sleep(7)
            self.servo.angle = self.current_angle
            time.sleep(0.2)

            self.turning = False
        "get boolean from camera, if camera sends True, the set steer so it would turn "
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = ServoSub()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

    
