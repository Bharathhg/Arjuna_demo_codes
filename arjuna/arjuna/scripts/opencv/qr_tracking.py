
#!/usr/bin/env python3

"""
Arjuna QR Tracking (Updated for Raw Image Subscription)
Detects QR code with "Arjuna" text, displays zones, publishes tracking commands

Company: NEWRRO TECH LLP
Website: www.newrro.in
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Changed from CompressedImage to Image
from std_msgs.msg import String
from cv_bridge import CvBridge     # Added for handling raw ROS2 images
import cv2
import numpy as np
import os
import time

# try import pyzbar for fallback multi-QR detection
try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except Exception:
    HAS_PYZBAR = False

# ==================== PARAMETERS ====================
WINDOW_NAME = "Arjuna QR Tracking"
TARGET_QR_TEXT = "Arjuna"  # Case sensitive
ZONE_LEFT_PERCENT = 40     # Left zone: 0-40%
ZONE_CENTER_PERCENT = 20   # Center zone: 40-60%
ZONE_RIGHT_PERCENT = 40    # Right zone: 60-100%

# Colors (BGR format)
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (0, 0, 255)
COLOR_BLUE = (255, 0, 0)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)

class QRTracker(Node):
    def __init__(self):
        super().__init__('arjuna_qr_tracker')
        
        # State
        self.current_frame = None
        self.qr_detected = False
        self.qr_zone = None  # 'left', 'center', 'right'
        self.qr_data = None
        self.qr_bbox = None
        self.is_running = True
        
        # Statistics
        self.frame_count = 0
        self.qr_count = 0
        self.start_time = time.time()
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Check for X display server
        self.show_window = "DISPLAY" in os.environ
        
        # QR Detector (OpenCV fallback)
        self.detector = cv2.QRCodeDetector()
        
        # Subscribe to camera (Updated to match okd_camera_pub.py)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publish QR tracking command
        self.qr_cmd_pub = self.create_publisher(
            String,
            '/arjuna/qr_tracking/command',
            10)
        
        if self.show_window:
            try:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            except Exception as e:
                self.get_logger().warn(f"Failed to create window: {e}. Running headless.")
                self.show_window = False
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARJUNA QR TRACKING - ACTIVE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Target QR: '{TARGET_QR_TEXT}' (case sensitive)")
        self.get_logger().info(f"Zones: LEFT {ZONE_LEFT_PERCENT}% | CENTER {ZONE_CENTER_PERCENT}% | RIGHT {ZONE_RIGHT_PERCENT}%")
        self.get_logger().info(f"pyzbar: {'AVAILABLE' if HAS_PYZBAR else 'NOT AVAILABLE (using OpenCV fallback)'}")
        self.get_logger().info(f"Display window: {'ENABLED' if self.show_window else 'DISABLED'}")
        self.get_logger().info("Subscribes: /camera/image_raw")
        self.get_logger().info("Publishes: /arjuna/qr_tracking/command")
        self.get_logger().info("=" * 60)
        if self.show_window:
            self.get_logger().info("Window: Green box = centered | Red box = need alignment")
            self.get_logger().info("Press Q or ESC to quit")
            self.get_logger().info("=" * 60)
        self.get_logger().info("")
    
    def image_callback(self, msg):
        """Process incoming camera frames"""
        try:
            # Decode raw image using CvBridge
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if frame is None:
                return
            
            self.frame_count += 1
            
            # Process frame for QR detection
            self.process_frame(frame)
            
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")
    
    def process_frame(self, frame):
        """Detect QR codes and determine zone"""
        height, width = frame.shape[:2]
        
        # Calculate zone boundaries
        left_boundary = int(width * ZONE_LEFT_PERCENT / 100)
        right_boundary = int(width * (ZONE_LEFT_PERCENT + ZONE_CENTER_PERCENT) / 100)
        
        # Draw zone partition lines
        cv2.line(frame, (left_boundary, 0), (left_boundary, height), COLOR_YELLOW, 2)
        cv2.line(frame, (right_boundary, 0), (right_boundary, height), COLOR_YELLOW, 2)
        
        # Add zone labels
        cv2.putText(frame, "LEFT", (int(left_boundary/2) - 30, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_YELLOW, 2)
        cv2.putText(frame, "CENTER", (left_boundary + int((right_boundary - left_boundary)/2) - 40, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_YELLOW, 2)
        cv2.putText(frame, "RIGHT", (right_boundary + int((width - right_boundary)/2) - 35, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_YELLOW, 2)
        
        # Reset detection state
        self.qr_detected = False
        self.qr_zone = None
        command = "stop"
        found_target = False
        
        # 1. Try pyzbar if available
        if HAS_PYZBAR:
            try:
                qr_codes = pyzbar.decode(frame)
            except Exception as e:
                self.get_logger().warn(f"pyzbar decode exception: {e}")
                qr_codes = []
            
            for qr in qr_codes:
                try:
                    qr_data = qr.data.decode('utf-8')
                except Exception:
                    continue
                
                # Only process QR with target text (case sensitive)
                if qr_data == TARGET_QR_TEXT:
                    self.qr_detected = True
                    self.qr_data = qr_data
                    self.qr_count += 1
                    found_target = True
                    
                    # Get QR bounding box
                    points = qr.polygon
                    if points is not None and len(points) == 4:
                        # Calculate center of QR code
                        qr_center_x = int(sum([p.x for p in points]) / 4)
                        qr_center_y = int(sum([p.y for p in points]) / 4)
                        
                        # Determine which zone QR is in
                        if qr_center_x < left_boundary:
                            self.qr_zone = "left"
                            box_color = COLOR_RED
                            command = "left"
                        elif qr_center_x < right_boundary:
                            self.qr_zone = "center"
                            box_color = COLOR_GREEN
                            command = "center"
                        else:
                            self.qr_zone = "right"
                            box_color = COLOR_RED
                            command = "right"
                        
                        # Draw bounding box
                        pts = np.array([[p.x, p.y] for p in points], np.int32)
                        pts = pts.reshape((-1, 1, 2))
                        cv2.polylines(frame, [pts], True, box_color, 3)
                        
                        # Draw center point
                        cv2.circle(frame, (qr_center_x, qr_center_y), 5, box_color, -1)
                        
                        # Display QR data and zone
                        text_y = qr_center_y - 20
                        cv2.putText(frame, f"QR: {qr_data}", 
                                  (qr_center_x - 50, text_y),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                        cv2.putText(frame, f"Zone: {self.qr_zone.upper()}", 
                                  (qr_center_x - 50, text_y + 25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                        
                        # Only process first matching QR
                        break
        
        # 2. Fallback to OpenCV QRCodeDetector
        if not found_target:
            try:
                qr_data, points, _ = self.detector.detectAndDecode(frame)
            except Exception as e:
                self.get_logger().debug(f"detectAndDecode exception: {e}")
                qr_data, points = None, None
            
            if qr_data == TARGET_QR_TEXT:
                self.qr_detected = True
                self.qr_data = qr_data
                self.qr_count += 1
                found_target = True
                
                if points is not None and len(points) > 0:
                    try:
                        points = points.astype(int).reshape((-1, 2))
                        if len(points) == 4:
                            # Calculate center of QR code
                            qr_center_x = int(sum([p[0] for p in points]) / 4)
                            qr_center_y = int(sum([p[1] for p in points]) / 4)
                            
                            # Determine which zone QR is in
                            if qr_center_x < left_boundary:
                                self.qr_zone = "left"
                                box_color = COLOR_RED
                                command = "left"
                            elif qr_center_x < right_boundary:
                                self.qr_zone = "center"
                                box_color = COLOR_GREEN
                                command = "center"
                            else:
                                self.qr_zone = "right"
                                box_color = COLOR_RED
                                command = "right"
                            
                            # Draw bounding box
                            cv2.polylines(frame, [points.reshape((-1, 1, 2))], True, box_color, 3)
                            
                            # Draw center point
                            cv2.circle(frame, (qr_center_x, qr_center_y), 5, box_color, -1)
                            
                            # Display QR data and zone
                            text_y = qr_center_y - 20
                            cv2.putText(frame, f"QR: {qr_data}", 
                                      (qr_center_x - 50, text_y),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                            cv2.putText(frame, f"Zone: {self.qr_zone.upper()}", 
                                      (qr_center_x - 50, text_y + 25),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    except Exception as e:
                        self.get_logger().warn(f"Failed to draw OpenCV QR bounding box: {e}")
        
        # Publish tracking command
        cmd_msg = String()
        cmd_msg.data = command
        self.qr_cmd_pub.publish(cmd_msg)
        
        # Add status info at bottom
        status_text = f"Status: {'QR DETECTED' if self.qr_detected else 'NO QR'} | Command: {command.upper()}"
        status_color = COLOR_GREEN if self.qr_detected else COLOR_RED
        cv2.putText(frame, status_text, (10, height - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Add FPS
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, height - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_WHITE, 2)
        
        # Display GUI (Main thread execution)
        if self.show_window:
            try:
                cv2.imshow(WINDOW_NAME, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q') or key == 27:
                    self.get_logger().info("Quit key pressed")
                    self.is_running = False
            except Exception as e:
                self.get_logger().warn(f"GUI display error: {e}")
    
    def print_stats(self):
        """Print statistics"""
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        zone_str = self.qr_zone.upper() if (self.qr_detected and self.qr_zone) else 'NONE'
        self.get_logger().info(
            f"Stats | Frames: {self.frame_count} | QR Detections: {self.qr_count} | "
            f"FPS: {fps:.1f} | Current: {'QR=' + zone_str if self.qr_detected else 'NO QR'}"
        )
    
    def cleanup(self):
        """Cleanup resources"""
        self.is_running = False
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except:
                pass
        self.get_logger().info("QR Tracker cleaned up")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tracker = QRTracker()
        
        while rclpy.ok() and tracker.is_running:
            rclpy.spin_once(tracker, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'tracker' in locals():
            tracker.cleanup()
            tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

