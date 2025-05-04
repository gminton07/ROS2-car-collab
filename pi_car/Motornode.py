#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class motorNode(Node):
    """
    A ROS 2 node that subscribes to camera Bool input,
    and publishes motor control commands to topic_motor.
    """

    def __init__(self):
        super().__init__('motor_command_publisher')

        self.duty = 70.0   # Speed (duty cycle)
        self.nod = 0.0     # Placeholder for nod (not used)
        self.go_dir = 1.0  # Forward
        self.stop_dir = 0.0

        # Publisher to the motor controller
        self.motor_pub = self.create_publisher(String, 'topic_motor', 10)

        # Subscriber to the camera signal
        self.subscription = self.create_subscription(
            Bool,
            'topic_camera',
            self.camera_callback,
            10
        )

        self.get_logger().info("Camera listener started — will publish to topic_motor")

    def camera_callback(self, msg):
        """
        If camera sees obstacle (True): stop.
        If clear (False): go forward.
        """
        if msg.data:
            self.get_logger().info("Obstacle detected — publishing STOP")
            command = f"{self.nod} 0.0 {self.stop_dir}"  # Duty 0%, direction 0
        else:
            self.get_logger().info("Path clear — publishing GO")
            command = f"{self.nod} {self.duty} {self.go_dir}"  # Duty 70%, direction 1

        motor_msg = String()
        motor_msg.data = command
        self.motor_pub.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    node = motorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
