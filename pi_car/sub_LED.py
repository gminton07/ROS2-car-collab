import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
import time
from threading import Thread

class RGB_ctrl(Node, Thread):
    left_R, left_G, left_B = (22, 23, 24)
    right_R, right_G, right_B = (10, 9, 25)

    Lpins, Rpins = (None, None)
    L_pwmR, L_pwmG, L_pwmB = (None, None, None)
    R_pwmR, R_pwmG, R_pwmB = (None, None, None)
    intersect, red, obst = (None, None, None)


    def __init__(self, Node, Thread):
        super().__init__('rgb_ctrl')
        Thread.__init__(self)

        # Set up LEDs
        self.setupLED(self.left_R, self.left_G, self.left_B,
                self.right_R, self.right_G, self.right_B)

        self.subscription = self.create_subscription(
                Float32,
                'lane_steering_angle',
                self.listener_angle,
                10)
        self.subscription = self.create_subscription(
                Bool,
                'intersection_detected',
                self.listener_intersection,
                10)
        self.subscription = self.create_subscription(
                Bool,
                'topic_red_detected',
                self.listener_red,
                10)
        self.subscription = self.create_subscription(
                Bool,
                'topic_obstacle',
                self.listener_obstacle,
                10)
        self.subscription
        self.get_logger().info('Initialized LED node.')

    def listener_angle(self, msg):
        self.angle = str(msg.data).split()
        self.angle = float(angle)

    def listener_intersection(self, msg):
        intersect = str(msg.data).split()
        self.intersect = bool(intersect)
        '''
        if self.intersect:   self.start(0xffff00)
        '''

    def listener_red(self, msg):
        red = str(msg.data).split()
        self.red = bool(red)
        '''
        if red:
            self.setColor(0xff0000, 'B')
        self.get_logger().info('brake lights')
        '''

    def listener_obstacle(self, msg):
        obst = str(msg.data).split()
        obst = bool(obst)
        '''
        if obst:
            self.start(0xff0000)
        '''

    # LED Control functions
    def setupLED(self, L_Rpin, L_Gpin, L_Bpin, R_Rpin, R_Gpin, R_Bpin):
        self.Lpins = {'pin_R':L_Rpin, 'pin_G':L_Gpin, 'pin_B':L_Bpin}
        self.Rpins = {'pin_R':R_Rpin, 'pin_G':R_Gpin, 'pin_B':R_Bpin}

        for i in self.Lpins:
            GPIO.setup(self.Lpins[i], GPIO.OUT)
            GPIO.output(self.Lpins[i], GPIO.HIGH)    # HIGH == off
        for i in self.Rpins:
            GPIO.setup(self.Rpins[i], GPIO.OUT)
            GPIO.output(self.Rpins[i], GPIO.HIGH)    # HIGH == off

        # Initialize left LED
        self.L_pwmR = GPIO.PWM(self.Lpins['pin_R'], 5000)
        self.L_pwmG = GPIO.PWM(self.Lpins['pin_G'], 5000)
        self.L_pwmB = GPIO.PWM(self.Lpins['pin_B'], 5000)
        self.L_pwmR.start(0)
        self.L_pwmG.start(0)
        self.L_pwmB.start(0)

        # Initialize right LED
        self.R_pwmR = GPIO.PWM(self.Rpins['pin_R'], 5000)
        self.R_pwmG = GPIO.PWM(self.Rpins['pin_G'], 5000)
        self.R_pwmB = GPIO.PWM(self.Rpins['pin_B'], 5000)
        self.R_pwmR.start(0)
        self.R_pwmG.start(0)
        self.R_pwmB.start(0)

        self.setColor(0x000000, 'L')
        self.setColor(0x000000, 'R')

    def map(self, x, in_min, in_max, out_min, out_max):
        return 100 - (x-in_min) / (in_max-in_min) * (out_max-out_min) + out_min

    def setColor(self, col, side):
        # side chooses which LED is changed
        # 'L' left, 'R' right, 'B' both

        R_val = (col & 0xff0000) >> 16
        G_val = (col & 0x00ff00) >> 8
        B_val = (col & 0x0000ff) >> 0

        R_val = self.map(R_val, 0, 255, 0, 100)
        G_val = self.map(G_val, 0, 255, 0, 100)
        B_val = self.map(B_val, 0, 255, 0, 100)
        
        if ('B' == side):
            print('both sides LED')
            self.L_pwmR.ChangeDutyCycle(R_val)
            self.L_pwmG.ChangeDutyCycle(G_val)
            self.L_pwmB.ChangeDutyCycle(B_val)
            self.R_pwmR.ChangeDutyCycle(R_val)
            self.R_pwmG.ChangeDutyCycle(G_val)
            self.R_pwmB.ChangeDutyCycle(B_val)
        elif ('L' == side):
            print('left side LED')
            self.L_pwmR.ChangeDutyCycle(R_val)
            self.L_pwmG.ChangeDutyCycle(G_val)
            self.L_pwmB.ChangeDutyCycle(B_val)
        elif ('R' == side):
            print('right side LED')
            self.R_pwmR.ChangeDutyCycle(R_val)
            self.R_pwmG.ChangeDutyCycle(G_val)
            self.R_pwmB.ChangeDutyCycle(B_val)
    
    def blink(self, col, tim, side):
        start_time = time.time()
        cur_time = start_time
        delay = 0.5
        i = 1

        while (start_time + tim > cur_time):
            cur_time = time.time()

            if (start_time + i * delay < cur_time):
                if i%2 == 0:
                    self.setColor(0x000000, side)
                else:
                    self.setColor(col, side)
                
                i += 1

    def off(self):
        for i in self.pins:
            GPIO.output(self.pins[i], GPIO.HIGH)    # Tied high is off

    def destroy(self):
        self.setColor(0x000000, 'B')
        self.pwmR.stop()
        self.pwmG.stop()
        self.pwmB.stop()
        #self.off()

    def run(self):
        while True:
            if self.obst:
                self.blink(0xff0000, 5, 'B')
                print('intersect blink yellow')
            elif self.red:
                pass
                self.setColor(0xff0000, 'B')
                print('red solid red')
            elif self.intersect:
                pass
                self.blink(0xffff00, 5, 'B')
                print('obstacle blink red')
            else:
                pass
        #self.blink(col, 5, 'B')


def main(args=None):
    rclpy.init(args=args)
    rgb_ctrl = RGB_ctrl(Node, Thread)

    #rgb_ctrl.off()

    rgb_ctrl.start()
    #print('Blinking')
    rclpy.spin(rgb_ctrl)

    rgb_ctrl.destroy()
    GPIO.cleanup()
    rgb_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
