import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String, Bool

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.subscription = self.create_subscription(
                String,
                'topic_motor',
                self.listener_motor,
                10)

        self.subscription = self.create_subscription(
            Bool,
            'topic_red_detected',
            self.red_callback,
            10
        )

        self.subscription = self.create_subscription(
            Bool,
            'intersection_detected',
            self.intersection_callback,
            10
        )
        self.subscription   # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')

        self.en = 11
        self.in1 = 13
        self.in2 = 12
        self.pwm_pin = self.motor_init(self.in1, self.in2, self.en, 1000, 50)    # Change starting DC
        self.stop = False
        self.intersection = False


    def motor_init(self, in1, in2, en, freq, dutyCycle):
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2,GPIO.OUT)
        GPIO.setup(en,GPIO.OUT)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        pwm_pin=GPIO.PWM(en,freq)
        pwm_pin.start(dutyCycle)
        return pwm_pin

    def motor_direction(self, in1, in2, direction, debug=False):
        # direction: -1 backward, 0 stop, 1 forward
        if (direction < 0):
            if debug: print('Set backward')
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif (direction == 0):
            if debug: print('Stopped')
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        elif (direction > 0):
            if debug: print('Set forwards')
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)

    def listener_motor(self, msg):
        if self.stop:
            dutyCycle = 0
            direction = 0
        else:
            [dutyCycle, direction] = str(msg.data).split()     # TODO: Change to preferred variables
            dutyCycle = float(dutyCycle)
            direction = int(direction)
            # TODO: Make smarter to only call motor_direction when direction value changes?
            self.motor_direction(self.in1, self.in2, direction)
            self.pwm_pin.ChangeDutyCycle(dutyCycle)
        self.get_logger().info(f'duty cycle: {dutyCycle}\tdirection: {direction}')

    def intersection_callback(self, msg):
        if msg.data:
            self.get_logger().info('Intersection detected')
            self.intersection = True
        else:
            self.get_logger().info('No intersection')
            self.intersection = False

    def red_callback(self, msg):
        if self.intersection and msg.data:
            self.get_logger().info('Stop Light detected, stopping motor')
            self.stop = True
        else:
            self.get_logger().info('No stop light')
            self.stop = False
    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy node explicitly
    # (optional - otherwise done automatically
    # when garbage collector comes
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
 
    main()

