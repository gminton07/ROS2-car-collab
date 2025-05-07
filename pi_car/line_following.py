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
        
        self.subscription = self.create_subscription(
            Image,
            'topic_camera',
            self.image_callback,
            10)
        
        self.angle_pub = self.create_publisher(Float32, 'lane_steering_angle', 10)
        self.intersection_pub = self.create_publisher(Bool, 'intersection_detected', 10)
        self.debug_pub = self.create_publisher(Image, 'lane_debug_image', 10)
        
        self.bridge = CvBridge()
        self.debug_mode = True
        
        # Processing parameters
        self.canny_low = 50
        self.canny_high = 150
        self.hough_threshold = 90
        self.hough_min_line_length = 20
        self.hough_max_line_gap = 30
        self.boundary_margin = 5
        
        # ROI parameters
        self.horizontal_split = 0.4
        self.left_horz_cut = 0.35
        self.left_vert_cut = 0.5
        self.right_horz_cut = 0.35
        self.right_vert_cut = 0.5
        
        # Intersection detection
        self.stop_line_roi_height = 0.3
        self.horizontal_angle_thresh = 10
        self.min_horizontal_length = 0.5
        self.detection_history_length = 5
        self.detection_threshold = 3
        
        self.intersection_detections = deque(maxlen=self.detection_history_length)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if cv_image is None or cv_image.size == 0:
                return
        except Exception as e:
            return

        steering_angle, intersection_detected, debug_img = self.process_image(cv_image)
        
        angle_msg = Float32()
        angle_msg.data = steering_angle
        self.angle_pub.publish(angle_msg)
        
        intersection_msg = Bool()
        intersection_msg.data = intersection_detected
        self.intersection_pub.publish(intersection_msg)
        
        if self.debug_mode and debug_img is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                self.debug_pub.publish(debug_msg)
            except:
                pass

    def create_roi_mask(self, height, width):
        mask = np.zeros((height+2, width+2), dtype=np.uint8)
        split_y = int(height * self.horizontal_split) + 1
        
        cv2.rectangle(mask, (1, split_y), (width, height+1), 255, -1)
        
        left_triangle = np.array([
            [1, split_y],
            [int(width * self.left_horz_cut)+1, split_y],
            [1, split_y + int((height-split_y)*self.left_vert_cut)]
        ])
        cv2.fillPoly(mask, [left_triangle], 0)
        
        right_triangle = np.array([
            [width-1, split_y],
            [width-int(width*self.right_horz_cut)-1, split_y],
            [width-1, split_y + int((height-split_y)*self.right_vert_cut)]
        ])
        cv2.fillPoly(mask, [right_triangle], 0)
        
        return mask[1:-1, 1:-1]

    def is_near_boundary(self, x, y, mask):
        if x < self.boundary_margin or x > mask.shape[1]-self.boundary_margin:
            return True
        if y < self.boundary_margin or y > mask.shape[0]-self.boundary_margin:
            return True
        return False

    def process_image(self, image):
        height, width = image.shape[:2]
        mask = self.create_roi_mask(height, width)
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        masked_blurred = cv2.bitwise_and(blurred, blurred, mask=mask)
        
        edges = cv2.Canny(masked_blurred, self.canny_low, self.canny_high)
        edges = cv2.bitwise_and(edges, edges, mask=mask)
        
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, self.hough_threshold,
                              minLineLength=self.hough_min_line_length,
                              maxLineGap=self.hough_max_line_gap)
        
        filtered_lines = []
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                if (mask[y1,x1] > 0 and mask[y2,x2] > 0 and
                    not self.is_near_boundary(x1,y1,mask) and
                    not self.is_near_boundary(x2,y2,mask)):
                    filtered_lines.append(line)
        
        lines = np.array(filtered_lines) if filtered_lines else None
        steering_angle = self.calculate_steering_angle(lines, width, height)
        
        stop_line_detected = self.detect_intersection(image, edges, mask)
        self.intersection_detections.append(stop_line_detected)
        confirmed_intersection = sum(self.intersection_detections) >= self.detection_threshold
        
        debug_img = None
        if self.debug_mode:
            debug_img = image.copy()
            if lines is not None:
                line_img = np.zeros_like(image)
                for line in lines:
                    x1,y1,x2,y2 = line[0]
                    cv2.line(line_img, (x1,y1), (x2,y2), (0,255,0), 2)
                debug_img = cv2.addWeighted(debug_img, 1, line_img, 1, 0)
                cv2.imwrite("detected_lines.png", line_img)
            
            split_y = int(height * self.horizontal_split)
            roi_overlay = np.zeros_like(image)
            cv2.rectangle(roi_overlay, (0,split_y), (width,height), (0,0,255), -1)
            
            left_triangle = np.array([
                [0, split_y],
                [int(width*self.left_horz_cut), split_y],
                [0, split_y + int((height-split_y)*self.left_vert_cut)]
            ])
            right_triangle = np.array([
                [width, split_y],
                [width-int(width*self.right_horz_cut), split_y],
                [width, split_y + int((height-split_y)*self.right_vert_cut)]
            ])
            cv2.fillPoly(roi_overlay, [left_triangle], (0,0,0))
            cv2.fillPoly(roi_overlay, [right_triangle], (0,0,0))
            
            cv2.addWeighted(roi_overlay, 0.2, debug_img, 0.8, 0, debug_img)
            
            cv2.putText(debug_img, f"Angle: {steering_angle:.1f}°", (10,30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.putText(debug_img, f"Intersection: {confirmed_intersection}", (10,60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        
        return steering_angle, confirmed_intersection, debug_img

    def detect_intersection(self, image, edges, mask):
        height, width = image.shape[:2]
        split_y = int(height * self.horizontal_split)
        roi_top = int(height - (height-split_y)*self.stop_line_roi_height)
        
        stop_mask = np.zeros_like(edges)
        cv2.rectangle(stop_mask, (0,roi_top), (width,height), 255, -1)
        masked_edges = cv2.bitwise_and(edges, edges, mask=stop_mask)
        
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 
                              self.hough_threshold//2,
                              self.hough_min_line_length//2,
                              self.hough_max_line_gap*2)
        
        if lines is None:
            return False
        
        min_length = width * self.min_horizontal_length
        for line in lines:
            x1,y1,x2,y2 = line[0]
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length < min_length:
                continue
            angle = np.arctan2(y2-y1, x2-x1) * 180/np.pi
            if abs(angle) < self.horizontal_angle_thresh or abs(angle-180) < self.horizontal_angle_thresh:
                return True
        return False

    def calculate_steering_angle(self, lines, image_width, image_height):
        if lines is None:
            return 0.0
        
        left_lines = []
        right_lines = []
        
        for line in lines:
            x1,y1,x2,y2 = line[0]
            slope = (y2-y1)/(x2-x1) if (x2-x1) != 0 else 0
            if slope < 0:
                left_lines.append((x1,y1,x2,y2))
            else:
                right_lines.append((x1,y1,x2,y2))
        
        if left_lines and right_lines:
            left_avg = np.mean([x for x1,y1,x2,y2 in left_lines for x in [x1,x2]])
            right_avg = np.mean([x for x1,y1,x2,y2 in right_lines for x in [x1,x2]])
            midpoint = (left_avg + right_avg)/2
            deviation = midpoint - (image_width/2)
            steering_angle = -(deviation/(image_width/2)) * 10
        elif left_lines:
            steering_angle = 10.0
        elif right_lines:
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