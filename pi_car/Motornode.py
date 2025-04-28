#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Node base class
from std_msgs.msg import Bool  # Message type used for camera signal (True/False)

import RPi.GPIO as GPIO  # Raspberry Pi GPIO library

# ----------------------------
# Motor Setup Functions
# ----------------------------

# Use BOARD numbering (physical pin numbers on Pi header)
GPIO.setmode(GPIO.BOARD)

def motor_init(in1, in2, en, freq, dutycycle):
    """
    Initializes the motor by setting up GPIO pins and PWM.

    Parameters:
    - in1, in2: Motor direction control pins (digital output)
    - en: Motor enable pin (PWM capable)
    - freq: Frequency of PWM
    - dutycycle: Initial duty cycle (0–100) to control speed

    Returns:
    - pwm_pin: The PWM object to control motor speed
    """
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(en, GPIO.OUT)

    # Make sure motor is initially stopped
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)

    # Start PWM for motor speed control
    pwm_pin = GPIO.PWM(en, freq)
    pwm_pin.start(dutycycle)

    return pwm_pin

def motor_direction(in1, in2, direction, debug=False):
    """
    Sets the direction of the motor.

    Parameters:
    - direction: -1 (backward), 0 (stop), 1 (forward)
    - debug: If True, prints debug messages
    """
    if direction < 0:
        if debug: print('Set backward')
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    elif direction == 0:
        if debug: print('Stopped')
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
    else:
        if debug: print('Set forward')
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)

# ----------------------------
# ROS 2 Node
# ----------------------------

class motorNode(Node):
    """
    A ROS 2 node that controls a motor based on camera input.

    - Subscribes to 'topic_camera' which sends Bool (True = stop, False = go).
    - Controls motor direction and speed using GPIO.
    """
    def __init__(self):
        super().__init__('minimal_subscriber')  # Initialize node with a name

        # Define motor control pin numbers (physical BOARD numbers)
        self.in1 = 11   # Direction control pin 1
        self.in2 = 13   # Direction control pin 2
        self.en = 15    # Enable pin (for PWM speed control)

        # PWM settings
        self.freq = 100       # PWM frequency in Hz
        self.duty = 70        # Initial duty cycle (% speed)

        # Initialize motor and PWM
        self.pwm = motor_init(self.in1, self.in2, self.en, self.freq, self.duty)

        # Start motor in forward direction
        motor_direction(self.in1, self.in2, 1)

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Bool,               # Message type
            'topic_camera',     # Topic name
            self.camera_callback,  # Callback function
            10                  # Queue size
        )

        self.get_logger().info("MinimalSubscriber (motor control) node started.")

    def camera_callback(self, msg):
        """
        Callback for when a new camera message is received.

        If msg.data is True → Stop motor.
        If msg.data is False → Start motor in forward direction.
        """
        if msg.data:
            self.get_logger().info("Camera triggered: Stopping motor.")
            motor_direction(self.in1, self.in2, 0)       # Stop motor
            self.pwm.ChangeDutyCycle(0)                  # Set speed to 0%
        else:
            self.get_logger().info("Camera clear: Running motor forward.")
            motor_direction(self.in1, self.in2, 1)       # Go forward
            self.pwm.ChangeDutyCycle(self.duty)          # Set speed back to original

    def destroy_node(self):
        """
        Cleanup when shutting down the node.
        """
        self.get_logger().info("Shutting down motor.")
        motor_direction(self.in1, self.in2, 0)  # Stop motor
        self.pwm.stop()                         # Stop PWM
        GPIO.cleanup()                          # Release GPIO resources
        super().destroy_node()                  # Clean up node


# ----------------------------
# Main Entry Point
# ----------------------------

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    minimal_subscriber = motorNode()  # Create node instance

    try:
        rclpy.spin(minimal_subscriber)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()  # Clean up
        rclpy.shutdown()                   # Shut down ROS 2


# Entry point when running the script
if __name__ == '__main__':
    main()
