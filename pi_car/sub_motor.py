import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
                String,
                'topic_motor',
                self.listener_motor,
                10)
        self.subscription   # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')

        self.in1 = 12
        self.in2 = 13
        self.en = 11
        self.pwm_pin = self.motor_init(self.in1, self.in2, self.en, 1000, 50)    # Change starting DC

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
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        elif (direction == 0):
            if debug: print('Stopped')
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        elif (direction > 0):
            if debug: print('Set forwards')
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)

    def listener_motor(self, msg):
        [dutyCycle, direction] = str(msg.data).split()     # TODO: Change to preferred variables
        dutyCycle = float(dutyCycle)
        direction = int(direction)
        # TODO: Make smarter to only call motor_direction when direction value changes?
        self.motor_direction(self.in1, self.in2, direction)
        self.pwm_pin.ChangeDutyCycle(dutyCycle)
        self.get_logger().info(f'duty cycle: {dutyCycle}\tdirection: {direction}')


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
