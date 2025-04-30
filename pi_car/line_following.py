#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from collections import deque

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Subscriber to camera feed
        self.subscription = self.create_subscription(
            Image,
            'topic_camera',
            self.image_callback,
            10)
        
        # Publisher for steering angle
        self.angle_pub = self.create_publisher(Float32, 'lane_steering_angle', 10)
        
        # Publisher for intersection detection
        self.intersection_pub = self.create_publisher(Bool, 'intersection_detected', 10)
        
        # Publisher for debug images
        self.debug_pub = self.create_publisher(Image, 'lane_debug_image', 10)
        
        self.bridge = CvBridge()
        self.debug_mode = True  # Enable debug by default
        
        # Image processing parameters
        self.canny_low = 50
        self.canny_high = 150
        self.hough_threshold = 50
        self.hough_min_line_length = 50
        self.hough_max_line_gap = 20
        
        # Region of interest parameters
        self.roi_bottom_width = 1.0
        self.roi_top_width = 0.6
        self.roi_height = 0.5
        
        # Intersection detection parameters
        self.stop_line_roi_height = 0.2  # Lower 20% of main ROI
        self.horizontal_angle_thresh = 10  # Degrees from horizontal to consider
        self.min_horizontal_length = 0.5   # Fraction of ROI width
        self.detection_history_length = 5  # Number of frames to consider
        self.detection_threshold = 3       # Minimum positive detections
        
        # Detection history buffer
        self.intersection_detections = deque(maxlen=self.detection_history_length)
        
        self.get_logger().info("Lane Detection Node initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")
            return

        # Process the image
        steering_angle, intersection_detected, debug_img = self.process_image(cv_image)
        
        # Publish the steering angle
        angle_msg = Float32()
        angle_msg.data = steering_angle
        self.angle_pub.publish(angle_msg)
        
        # Publish intersection detection
        intersection_msg = Bool()
        intersection_msg.data = intersection_detected
        self.intersection_pub.publish(intersection_msg)
        
        # Publish debug image if enabled
        if self.debug_mode and debug_img is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Debug image error: {str(e)}")

    def process_image(self, image):
        height, width = image.shape[:2]
        
        # Convert to grayscale and blur
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny edge detection
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # Create main ROI mask (trapezoid)
        mask = np.zeros_like(edges)
        bottom_left = (int(width * (1 - self.roi_bottom_width) / 2), height)
        bottom_right = (int(width * (1 + self.roi_bottom_width) / 2), height)
        top_left = (int(width * (1 - self.roi_top_width) / 2), int(height * (1 - self.roi_height)))
        top_right = (int(width * (1 + self.roi_top_width) / 2), int(height * (1 - self.roi_height)))
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Detect lines
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, self.hough_threshold,
                               minLineLength=self.hough_min_line_length,
                               maxLineGap=self.hough_max_line_gap)
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(lines, width, height)
        
        # Detect intersection (stop line)
        stop_line_detected = self.detect_intersection(image, edges, vertices)
        self.intersection_detections.append(stop_line_detected)
        
        # Require multiple detections to confirm intersection
        confirmed_intersection = sum(self.intersection_detections) >= self.detection_threshold
        
        # Create debug image
        debug_img = None
        if self.debug_mode:
            debug_img = image.copy()
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw ROI
            cv2.polylines(debug_img, vertices, True, (0, 0, 255), 2)
            
            # Draw stop line ROI
            stop_line_top = (bottom_left[0], int(bottom_left[1] - height * self.roi_height * self.stop_line_roi_height))
            stop_line_bottom = bottom_left[1]
            stop_line_vertices = np.array([
                [stop_line_top, (stop_line_top[0], stop_line_bottom),
                 (bottom_right[0], stop_line_bottom), (bottom_right[0], stop_line_top)]
                ])
            cv2.polylines(debug_img, [stop_line_vertices], True, (255, 0, 0), 2)
            
            # Add text overlay
            cv2.putText(debug_img, f"Angle: {steering_angle:.1f}°", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(debug_img, f"Intersection: {confirmed_intersection}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return steering_angle, confirmed_intersection, debug_img

    def detect_intersection(self, image, edges, main_vertices):
        height, width = image.shape[:2]
        
        # Define stop line ROI (bottom portion of main ROI)
        bottom_left = main_vertices[0][0]
        bottom_right = main_vertices[0][3]
        roi_top = int(bottom_left[1] - height * self.roi_height * self.stop_line_roi_height)
        
        # Create mask for stop line detection
        mask = np.zeros_like(edges)
        stop_line_vertices = np.array([[
            (bottom_left[0], roi_top),
            bottom_left,
            bottom_right,
            (bottom_right[0], roi_top)
        ]], dtype=np.int32)
        cv2.fillPoly(mask, stop_line_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Detect lines in stop line ROI
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 
                               threshold=self.hough_threshold // 2,  # Lower threshold for stop lines
                               minLineLength=self.hough_min_line_length // 2,
                               maxLineGap=self.hough_max_line_gap * 2)
        
        if lines is None:
            return False
        
        # Check for horizontal lines
        min_length = width * self.min_horizontal_length
        detected = False
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            if length < min_length:
                continue
                
            # Calculate angle (0° is horizontal)
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            if abs(angle) < self.horizontal_angle_thresh or abs(angle - 180) < self.horizontal_angle_thresh:
                detected = True
                break
                
        return detected

    def calculate_steering_angle(self, lines, image_width, image_height):
        pass
        # ... (keep the same implementation as before) ...

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
