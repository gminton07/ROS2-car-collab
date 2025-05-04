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
        self.angle_center = 0.0
        self.angle_left = -10.0
        self.angle_right = 10.0

        self.load_servo_config()

        # Initialize servo position to center
        self.current_angle = self.angle_center
        self.servo.angle = self.current_angle

        # Publisher to topic_servo
        self.servo_command_publisher = self.create_publisher(String, 'topic_servo', 10)

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
        config_file = 'PICAR_CONFIG.txt'
        if os.path.exists(config_file):
            self.get_logger().info('Servo config file found. Loading values...')
            try:
                with open(config_file, 'r') as f:
                    lines = f.readlines()
                    if len(lines) >= 9:
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
        if normalized_angle < 90.0:
            mapped_angle = self.angle_center - (90.0 - normalized_angle) / 90.0 * (self.angle_center - self.angle_left)
        else:
            mapped_angle = self.angle_center + (normalized_angle - 90.0) / 90.0 * (self.angle_right - self.angle_center)
        mapped_angle = max(self.angle_left, min(self.angle_right, mapped_angle))
        return mapped_angle

    def publish_servo_command(self):
        nod = 0.0
        swivel = 0.0
        steer = self.current_angle
        msg = String()
        msg.data = f"{nod} {swivel} {steer}"
        self.servo_command_publisher.publish(msg)
        self.get_logger().info(f"Published to topic_servo: {msg.data}")

    def listener_angle(self, msg):
        if not self.turning:
            line_angle = msg.data
            self.get_logger().info(f"Received lane steering angle: {line_angle}")

            mapped_angle = self.map_steering_angle(line_angle)
            self.get_logger().info(f"Mapped to servo angle: {mapped_angle}")

            self.servo.angle = mapped_angle
            self.current_angle = mapped_angle
            self.publish_servo_command()

    def listener_intersection(self, msg):
        if msg.data and not self.turning:
            self.get_logger().info("Intersection detected. Performing turn.")
            self.turning = True

            turn_angle = self.current_angle + 20 if self.current_angle < (self.angle_right - 20) else self.current_angle - 20
            self.servo.angle = turn_angle
            self.current_angle = turn_angle
            self.publish_servo_command()

            time.sleep(7)

            self.servo.angle = self.angle_center/home/gabe/.local/lib/python3.10/site-packages/adafruit_motor/servo.py
            self.current_angle = self.angle_center
            self.publish_servo_command()
            time.sleep(0.2)

            self.turning = False

    def listener_camera(self, msg):
        if msg.data and not self.turning:
            self.get_logger().info("Camera triggered evasive turn.")
            self.turning = True

            turn_angle = self.current_angle + 20 if self.current_angle < (self.angle_right - 20) else self.current_angle - 20
            self.servo.angle = turn_angle
            self.current_angle = turn_angle
            self.publish_servo_command()

            time.sleep(7)

            self.servo.angle = self.angle_center
            self.current_angle = self.angle_center
            self.publish_servo_command()
            time.sleep(0.2)

            self.turning = False

    def listener_keyboard(self, msg):
        self.get_logger().info('Keyboard input: %s' % msg.data)
        slight_turn_amount = 5

        if msg.data == 's':
            self.servo.angle = max(self.angle_left, self.current_angle - slight_turn_amount)
            self.current_angle = self.servo.angle
            self.get_logger().info(f"Slightly steering left: {self.servo.angle}")
        elif msg.data == 'd':
            self.servo.angle = min(self.angle_right, self.current_angle + slight_turn_amount)
            self.current_angle = self.servo.angle
            self.get_logger().info(f"Slightly steering right: {self.servo.angle}")
        elif msg.data == 'e':
            self.servo.angle = self.angle_center
            self.current_angle = self.angle_center
            self.get_logger().info(f"Centering: {self.angle_center}")
        
        self.publish_servo_command()


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
