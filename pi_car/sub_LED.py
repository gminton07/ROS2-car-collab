import rclpy
from rclpy.node inport Node

from std_msgs.msg Float32, Bool

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
import time, threading

class RGB_ctrl(Node, threading.Thread):
    left_R, left_G, left_B = (22, 23, 24)
    right_R, right_G, right_B = (10, 9, 25)

    pins = None
    pwmR, pwmG, pwmB = (None, None, None)

    def __init__(self):
        super().__init__('rgb_ctrl')

        # Set up LEDs

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
        self.subscription_red = self.create_subscription(
                Bool,
                'topic_red_detected',
                self.listener_red,
                10)
        self.subscription()

        self.get_logger().info('Initialized LED node.')

    def listener_angle():
        pass

    def listener_intersection():
        pass

    def listener_red():
        pass

    # LED Control functions
    def setupLED(self, Rpin, Gpin, Bpin):
        self.pins = {'pin_R':Rpin, 'pin_G':Gpin, 'pin_B':Bpin}

        for i in self.pins:
            GPIO.setup(self.pins[i], GPIO.OUT)
            GPIO.output(self.pins[i], GPIO.LOW)

        self.pwmR = GPIO.PWM(self.pins['pin_R'], 50)
        self.pwmG = GPIO.PWM(self.pins['pin_G'], 50)
        self.pwmB = GPIO.PWM(self.pins['pin_B'], 50)

        self.pwmR.start(0)
        self.pwmG.start(0)
        self.pwmB.start(0)

    def map(self, x, in_min, in_max, out_min, out_max):
        return 100 - (x-in_min) / (in_max-in_min) * (out_max-out_min) + out_min

    def setColor(self, col):
        R_val = (col & 0xff0000) >> 16
        G_val = (col & 0x00ff00) >> 8
        B_val = (col & 0x0000ff) >> 0

        R_val = self.map(R_val, 0, 255, 0, 100)
        G_val = self.map(G_val, 0, 255, 0, 100)
        B_val = self.map(B_val, 0, 255, 0, 100)

        self.pwmR.ChangeDutyCycle(R_val)
        self.pwmG.ChangeDutyCycle(G_val)
        self.pwmB.ChangeDutyCycle(B_val)

     def blink(self, col, tim):
        start_time = time.time()
        cur_time = start_time
        delay = 0.5
        i = 1

        while (start_time + tim > cur_time):
            cur_time = time.time()

            if (start_time + i * delay < cur_time):
                if i%2 == 0:
                    self.setColor(0x000000)
                    #self.destroy()
                else:
                    self.setColor(col)
                
                i += 1

    def off(self):
        for i in self.pins:
            GPIO.output(self.pins[i], GPIO.LOW)

    def destroy(self):
        self.pwmR.stop()
        self.pwmG.stop()
        self.pwmB.stop()
        for i in self.pins:
            GPIO.output(self.pins[i], GPIO.LOW)

    def run(self):
        self.blink(0xffff00, 5)


def main(args=None):
    rclpy.init(args=args)
    rgb_l = RGB_ctrl()
    rgb_l.setupLED(left_R, left_G, left_B)
    rgb_r = RGB_ctrl()
    rgb_r.setupLED(right_R, right_G, right_B)

    rgb_l.start()
    rgb_r.blink(0xffff00, 5)

    rclpy.spin(rgb_l)
    rclpy.spin(rgb_r)

    rgb_l.destroy()
    rgb_r.destroy()
    rgb_l.destroy_node()
    rgb_r.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
