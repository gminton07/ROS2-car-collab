import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

#import board
#from adafruit_pca9685 import PCA9685
#from adafruit_motor import servo 

import time
#import adafruit_mmc56x3
import os

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Default (if no config file found)
        self.angle_center = 0.0
        self.angle_left = -10.0
        self.angle_right = 10.0

        #self.load_servo_config()

        # Initialize servo position to center
        self.current_angle = self.angle_center
        self.servo_angle = self.current_angle

        # Publisher to topic_servo
        self.servo_command_publisher = self.create_publisher(String, 'topic_servo', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

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
            self.listener_red,
            10)
        
        self.subscription = self.create_subscription(
            String,
            'topic_keyboard',
            self.listener_keyboard,
            10)

        self.get_logger().info('Initialized the subscriber node.')

        self.turning = False  # For managing intersection turns
        self.line_turn = 'center'
        self.line_angle = 0
        self.mid, self.left, self.right = self.load_servo_config()

    def load_servo_config(self):
        config_file = '/home/gabe/ros2_ws/src/pi_car/pi_car/PICAR_CONFIG.txt'
        if os.path.exists(config_file):
            self.get_logger().info('Servo config file found. Loading values...')
            try:
                with open(config_file, 'r') as f:
                    lines = f.readlines()
                    if len(lines) >= 9:
                        mid = float(lines[6].strip())
                        left = float(lines[7].strip())
                        right = float(lines[8].strip())
                        self.get_logger().info(f'Loaded config: center={mid}, left={left}, right={right}')
                        return mid, left, right
                    else:
                        self.get_logger().warn('Config file found but not enough values.')
            except Exception as e:
                self.get_logger().error(f'Failed to load config: {e}')
        else:
            self.get_logger().info('No servo config found. Using defaults.')

    # If loading failed or file not found, return default values
            return 0.0, 0.0, 0.0  # Replace with your desired default values

    def map_steering_angle(self, angle, mid, left, right):
        if mid is None or left is None:
            self.get_logger().warn('Invalid servo calibration values.')
            return 0.0

        steer_angle = ((angle - mid) / (left - mid)) * 10
        return steer_angle

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
            self.line_angle = msg.data
            #slight_turn_amount = 2

            self.get_logger().info(f"Received lane steering angle: {self.line_angle}")
            if self.line_angle < 0:
                self.line_turn = 'left'
            elif self.line_angle > 0:
                self.line_turn = 'right'
            else:
                self.line_turn = 'center'
            
            '''
            #mapped_angle = self.map_steering_angle(line_angle, self.mid, self.left, self.right)
            #self.get_logger().info(f"Mapped to servo angle: {mapped_angle}")
            mapped_angle = max(-10, self.current_angle - slight_turn_amount)
            self.servo_angle = mapped_angle
            self.current_angle = mapped_angle
            self.publish_servo_command()
            '''
    def timer_callback(self):

        if self.line_turn == 'left':
            if self.current_angle > self.line_angle:
                self.current_angle -= 1
            else:
                self.current_angle = self.line_angle
        elif self.line_turn == 'right':
            if self.current_angle < self.line_angle:
                self.current_angle += 2
            else:
                self.current_angle = self.line_angle 
        elif self.line_turn == 'center':
            self.current_angle = 0
        else:            
            self.current_angle = 0
        self.publish_servo_command() 
        i =+ 1


    def listener_intersection(self, msg):
        if msg.data and not self.turning:
            self.get_logger().info("Intersection detected. Performing turn.")
            self.turning = True

            turn_angle = self.current_angle + 3 if self.current_angle < (self.angle_right - 3) else self.current_angle - 3
            self.servo_angle = turn_angle
            self.current_angle = turn_angle
            self.publish_servo_command()

            time.sleep(2)

            self.servo_angle = self.angle_center
            self.current_angle = self.angle_center
            self.publish_servo_command()
            time.sleep(0.2)

            self.turning = False

    def listener_red(self, msg):
        if msg.data and not self.turning:
            self.get_logger().info("Camera triggered evasive turn.")
            self.turning = True

            turn_angle = self.current_angle + 3 if self.current_angle < (self.angle_right - 3) else self.current_angle - 3
            self.servo_angle = turn_angle
            self.current_angle = turn_angle
            self.publish_servo_command()

            time.sleep(7)

            self.servo_angle = self.angle_center
            self.current_angle = self.angle_center
            self.publish_servo_command()
            time.sleep(0.2)

            self.turning = False

    def listener_keyboard(self, msg):
        self.get_logger().info('Keyboard input: %s' % msg.data)
        slight_turn_amount = 2

        if msg.data == 'a':
            self.servo_angle = max(-10, self.current_angle - slight_turn_amount)
            self.current_angle = self.servo_angle
            self.get_logger().info(f"Slightly steering left: {self.servo_angle}")
        elif msg.data == 'd':
            self.servo_angle = min(10, self.current_angle + slight_turn_amount)
            self.current_angle = self.servo_angle
            self.get_logger().info(f"Slightly steering right: {self.servo_angle}")
        elif msg.data == 's':
            self.servo_angle = 0
            self.current_angle = 0
            self.get_logger().info(f"Centering: 0")
        
        self.publish_servo_command()

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
