#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class DebugLineFollower(Node):
    def __init__(self):
        super().__init__('debug_line_follower')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image, 
            '/robot/camera/image_raw',
            self.image_callback, 
            10
        )
        
        # Parameters - TUNED FOR DEBUGGING
        self.linear_speed = 0.2  # Slower speed for better control
        self.angular_speed = 0.5
        
        # Image processing parameters
        self.image_width = 640
        self.image_height = 480
        self.roi_height = 100  # Smaller ROI for testing
        
        # Simple controller (no PID initially)
        self.error_threshold = 50  # Larger threshold
        
        # Debug variables
        self.frame_count = 0
        self.images_received = 0
        self.last_line_center = None
        
        # Status tracking
        self.camera_working = False
        self.line_detected = False
        
        self.get_logger().info('=== DEBUG LINE FOLLOWER STARTED ===')
        self.get_logger().info(f'Subscribing to: /robot/camera/image_raw')
        self.get_logger().info(f'Publishing to: /cmd_vel')
        
        # Status timer - report every 2 seconds
        self.status_timer = self.create_timer(2.0, self.status_callback)
        
        # Send initial movement command to test
        self.test_movement_timer = self.create_timer(1.0, self.test_movement)
        self.test_count = 0
        
    def test_movement(self):
        """Test basic movement for 10 seconds"""
        if self.test_count < 5:  # Test for 5 seconds
            twist = Twist()
            if self.test_count % 2 == 0:
                twist.linear.x = 0.1
                self.get_logger().info('TEST: Moving forward')
            else:
                twist.angular.z = 0.2
                self.get_logger().info('TEST: Turning')
            
            self.cmd_vel_pub.publish(twist)
            self.test_count += 1
        else:
            # Cancel the test timer after 5 iterations
            if hasattr(self, 'test_movement_timer'):
                self.test_movement_timer.cancel()
                self.get_logger().info('TEST: Basic movement test completed')
    
    def status_callback(self):
        """Print status information"""
        self.get_logger().info(f'=== STATUS ===')
        self.get_logger().info(f'Images received: {self.images_received}')
        self.get_logger().info(f'Camera working: {self.camera_working}')
        self.get_logger().info(f'Line detected: {self.line_detected}')
        if self.last_line_center:
            self.get_logger().info(f'Last line center: {self.last_line_center}')
        
    def image_callback(self, msg):
        self.images_received += 1
        self.camera_working = True
        self.frame_count += 1
        
        # Log every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Received frame {self.frame_count}, size: {msg.width}x{msg.height}, encoding: {msg.encoding}')
        
        try:
            # Convert ROS image to OpenCV format
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image
            line_center = self.process_image_simple(cv_image)
            
            # Simple control logic
            self.simple_line_following(line_center)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
            # Send stop command on error
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
    
    def process_image_simple(self, image):
        """Simplified image processing for debugging"""
        try:
            height, width = image.shape[:2]
            
            # Use bottom part of image
            roi_top = height - self.roi_height
            roi = image[roi_top:height, 0:width]
            
            # Convert to grayscale
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # Simple binary thresholding - look for dark pixels
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
            
            # Find contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get contour area
                area = cv2.contourArea(largest_contour)
                
                if area > 200:  # Minimum area threshold
                    # Calculate moments
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        self.line_detected = True
                        self.last_line_center = cx
                        
                        # Log detection every 30 frames
                        if self.frame_count % 30 == 0:
                            self.get_logger().info(f'Line detected at x={cx}, area={area}')
                        
                        return cx
            
            # No line detected
            self.line_detected = False
            return None
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
            return None
    
    def simple_line_following(self, line_center):
        """Simplified line following logic"""
        twist = Twist()
        
        image_center = self.image_width // 2
        
        if line_center is not None:
            # Calculate error
            error = line_center - image_center
            
            # Always move forward when line is detected
            twist.linear.x = self.linear_speed
            
            # Simple proportional control
            if abs(error) < self.error_threshold:
                # Go straight
                twist.angular.z = 0.0
                if self.frame_count % 30 == 0:
                    self.get_logger().info('Going STRAIGHT')
            elif error > 0:
                # Line is to the right, turn right
                twist.angular.z = -0.3
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Turning RIGHT (error: {error})')
            else:
                # Line is to the left, turn left  
                twist.angular.z = 0.3
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Turning LEFT (error: {error})')
        else:
            # No line detected - stop and search
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Slow rotation to search
            if self.frame_count % 30 == 0:
                self.get_logger().warning('NO LINE - Searching...')
        
        # Publish command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    line_follower = DebugLineFollower()
    
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        print("\nShutting down debug line follower...")
    finally:
        # Stop robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        line_follower.cmd_vel_pub.publish(twist)
        
        line_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()