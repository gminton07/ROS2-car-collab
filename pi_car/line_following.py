#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from collections import deque
import os

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
        self.hough_threshold = 90
        self.hough_min_line_length = 15
        self.hough_max_line_gap = 30
        
        # ROI parameters with adjustable triangle cuts
        self.horizontal_split = 0.5  # Vertical position to split image (0-1)
        # Left triangle cuts (as percentages)
        self.left_horz_cut = 0.35  # Horizontal cut percentage
        self.left_vert_cut = 0.5  # Vertical cut percentage
        # Right triangle cuts (as percentages)
        self.right_horz_cut = 0.35  # Horizontal cut percentage
        self.right_vert_cut = 0.5  # Vertical cut percentage
        
        # Intersection detection parameters
        self.stop_line_roi_height = 0.3
        self.horizontal_angle_thresh = 10
        self.min_horizontal_length = 0.5
        self.detection_history_length = 5
        self.detection_threshold = 3
        
        # Detection history buffer
        self.intersection_detections = deque(maxlen=self.detection_history_length)
        
        self.get_logger().info("Lane Detection Node initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if cv_image is None or cv_image.size == 0:
                self.get_logger().warn("Received empty image, skipping processing")
                return
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

    def create_roi_mask(self, height, width):
        """Create ROI mask with adjustable triangular cuts"""
        mask = np.zeros((height, width), dtype=np.uint8)
        
        # Calculate split position
        split_y = int(height * self.horizontal_split)
        
        # Create base rectangle (lower portion)
        cv2.rectangle(mask, (0, split_y), (width, height), 255, -1)
        
        # Cut left triangle
        left_triangle = np.array([
            [0, split_y],
            [int(width * self.left_horz_cut), split_y],
            [0, split_y + int((height - split_y) * self.left_vert_cut)]
        ])
        cv2.fillPoly(mask, [left_triangle], 0)
        
        # Cut right triangle
        right_triangle = np.array([
            [width, split_y],
            [width - int(width * self.right_horz_cut), split_y],
            [width, split_y + int((height - split_y) * self.right_vert_cut)]
        ])
        cv2.fillPoly(mask, [right_triangle], 0)
        
        return mask

    def process_image(self, image):
        height, width = image.shape[:2]
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Canny edge detection
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)
        
        # Create ROI mask with adjustable triangles
        mask = self.create_roi_mask(height, width)
        masked_edges = cv2.bitwise_and(edges, edges, mask=mask)
        
        # Detect lines
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, self.hough_threshold,
                              minLineLength=self.hough_min_line_length,
                              maxLineGap=self.hough_max_line_gap)
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(lines, width, height)
        
        # Detect intersection
        stop_line_detected = self.detect_intersection(image, edges, mask)
        self.intersection_detections.append(stop_line_detected)
        confirmed_intersection = sum(self.intersection_detections) >= self.detection_threshold
        
        # Create debug image
        debug_img = None
        if self.debug_mode:
            debug_img = image.copy()
            
            # Draw detected lines
            if lines is not None:
                line_img = np.zeros_like(image)
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                debug_img = cv2.addWeighted(debug_img, 1, line_img, 1, 0)
                
                # Save lines image
                cv2.imwrite(os.path.join(os.path.dirname(__file__), "detected_lines.png"), line_img)
            
            # Draw ROI overlay
            roi_overlay = np.zeros_like(image)
            split_y = int(height * self.horizontal_split)
            cv2.rectangle(roi_overlay, (0, split_y), (width, height), (0, 0, 255), -1)
            
            # Draw triangles
            left_triangle = np.array([
                [0, split_y],
                [int(width * self.left_horz_cut), split_y],
                [0, split_y + int((height - split_y) * self.left_vert_cut)]
            ])
            right_triangle = np.array([
                [width, split_y],
                [width - int(width * self.right_horz_cut), split_y],
                [width, split_y + int((height - split_y) * self.right_vert_cut)]
            ])
            cv2.fillPoly(roi_overlay, [left_triangle], (0, 0, 0))
            cv2.fillPoly(roi_overlay, [right_triangle], (0, 0, 0))
            
            cv2.addWeighted(roi_overlay, 0.2, debug_img, 0.8, 0, debug_img)
            
            # Add text overlay
            cv2.putText(debug_img, f"Angle: {steering_angle:.1f}°", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(debug_img, f"Intersection: {confirmed_intersection}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return steering_angle, confirmed_intersection, debug_img

    def detect_intersection(self, image, edges, main_mask):
        height, width = image.shape[:2]
        split_y = int(height * self.horizontal_split)
        
        # Define stop line ROI (bottom portion of main ROI)
        roi_top = int(height - (height - split_y) * self.stop_line_roi_height)
        
        # Create mask for stop line detection
        mask = np.zeros_like(edges)
        cv2.rectangle(mask, (0, roi_top), (width, height), 255, -1)
        masked_edges = cv2.bitwise_and(edges, edges, mask=mask)
        
        # Detect lines
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 
                              threshold=self.hough_threshold // 2,
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
                
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            if abs(angle) < self.horizontal_angle_thresh or abs(angle - 180) < self.horizontal_angle_thresh:
                detected = True
                break
                
        return detected

    def calculate_steering_angle(self, lines, image_width, image_height):
        if lines is None:
            return 0.0
        
        left_lines = []
        right_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            
            if slope < 0:  # Left lane
                left_lines.append(parameters)
            else:  # Right lane
                right_lines.append(parameters)
        
        left_avg = np.average(left_lines, axis=0) if left_lines else None
        right_avg = np.average(right_lines, axis=0) if right_lines else None
        
        if left_avg is not None and right_avg is not None:
            left_x = (image_height - left_avg[1]) / left_avg[0]
            right_x = (image_height - right_avg[1]) / right_avg[0]
            midpoint = (left_x + right_x) / 2
            deviation = midpoint - (image_width / 2)
            steering_angle = -(deviation / (image_width / 2)) * 10
        elif left_avg is not None:
            steering_angle = 10.0
        elif right_avg is not None:
            steering_angle = -10.0
        else:
            steering_angle = 0.0
        
        return max(-10.0, min(10.0, steering_angle))

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()