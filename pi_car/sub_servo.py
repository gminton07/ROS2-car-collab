import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import board
import Adafruit_PCA9685 as PWM_HAT
import os.path

class MinimalSubscriber(Node):

    # Global variables
    PICAR_CONFIG_FILE_NAME = 'PICAR_CONFIG.txt'
    SERVO_NOD, SERVO_SWIVEL, SERVO_STEER = (0, 1, 2)



    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
                String,
                'topic_servo',
                self.listener_servo,
                10)
        self.subscription   # prevent unused variable warning
        self.get_logger().info('I initialized the subscriber')

        self._servo_global_pwm = PWM_HAT.PCA9685()
        self._servo_global_pwm.set_pwm_freq(60)

        # TODO: only initializing NOD, come back for rest later...
        _servo_global_pwm = None
        #_servo_nod_pin, _servo_nod_pwm = (None, None)
        _servo_nod_left, _servo_nod_middle, _servo_nod_right = (290, 310, 330)
        nod_servo_state = 0

        # Get config file settings
        if os.path.exists(PICAR_CONFIG_FILE_NAME):
            print('servo configuration found!')
            with open(self.PICAR_CONFIG_FILE_NAME, 'r') as config:
                configuration = config.readlines()
                if len(configuration < 9):
                    print('invalid configuration')
                else:
                    self.configure_nod_servo_positions(
                            int(configuration[1]),
                            int(configuration[0]),
                            int(configuration[2]),
                            )
        #self._servo_nod_pin = SERVO_NOD

    def _calc_servo_duty_cycle(self, left, middle, right, amount, is_left):
        return (
                middle - (middle - left) * amount / 10
                if is_left
                else (right - middle) * amount / 10 + middle)

    def _set_servo(self, servo, value=None):
        safe_value = max(min(10, value), -10)
        if safe_value != value:
            print('Warning! Value outside expected range [-10, 10]')
            value = safe_value

        is_left, amount = (value < 0, abs(value))
        duty_cycle = None
        if servo == SERVO_NOD:
            duty_cycle = self._calc_servo_duty_cycle(
                    self._servo_nod_left,
                    self._servo_nod_middle,
                    self._servo_nod_right,
                    amount,
                    is_left,)
            self._servo_global_pwm.set_pwm(SERVO_NOD, 0, int(duty_cycle))

    def configure_nod_servo_positions(self, left=None, middle=None, right=None):
        self._servo_nod_left = left
        self._servo_nod_middle = middle
        self._servo_nod_right = right
        

    # TODO: Get servo updater code

    def listener_servo(self, msg):
        [nod, swivel, steer] = str(msg.data).split()
        nod = float(nod)
        swivel = float(swivel)
        steer = float(steer)

        self.get_logger().info(f'steer: {steer}\tswivel:{swivel}\tnod:{nod}')

        # nod 0, swivel 1, steer 2
        self._set_servo(0, nod)


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
