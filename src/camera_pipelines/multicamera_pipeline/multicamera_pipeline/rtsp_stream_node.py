#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import threading
import time
import signal
import sys
import subprocess
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class RtspStreamNode(Node):
    def __init__(self):
        super().__init__('rtsp_stream_node')
        
        # Set logging level to WARN to only show warnings and errors
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        
        # Declare and get only necessary parameters
        self.declare_parameter('rtsp_url', 'rtsp://192.168.2.250:554/stream')
        self.declare_parameter('frame_rate', 20.0)  # Hz, updated to match camera specs
        self.declare_parameter('image_width', 800)  # Match actual stream resolution
        self.declare_parameter('image_height', 448)  # Match actual stream resolution
        self.declare_parameter('topic_name', 'camera/image_raw')
        self.declare_parameter('pipeline_type', 4)  # Use TCP pipeline by default
        
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.pipeline_type = self.get_parameter('pipeline_type').get_parameter_value().integer_value
        
        # Create QoS profile for reliable communication with volatile durability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE  # Most compatible with subscribers
        )
        
        # Create publisher with reliable QoS settings
        self.publisher = self.create_publisher(Image, self.topic_name, qos_profile)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize capture thread
        self.is_running = False
        self.capture_thread = None
        
        # Check for hardware acceleration
        self.has_vaapi = self._check_for_vaapi()
        self.get_logger().info(f"Hardware acceleration (VAAPI): {'Available' if self.has_vaapi else 'Not available'}")
        
        # Create a timer for diagnostics (only log if there are issues)
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        # Performance metrics
        self.frame_count = 0
        self.last_log_time = time.time()
        self.last_fps = 0.0
        
        # Video capture object
        self.cap = None
        
        # Flush any buffered frames on the network interface for this camera
        self._flush_network_buffer()
        
        # Start capturing frames
        self.start_capture()
        
        self.get_logger().info(f"RTSP Stream Node initialized with URL: {self.rtsp_url}")
        self.get_logger().info(f"Publishing to topic: {self.topic_name}")
        self.get_logger().info(f"Using pipeline type: {self.pipeline_type}")
        self.get_logger().info("Ultra-low latency mode enabled")

    def _check_for_vaapi(self):
        """Check if VAAPI hardware acceleration is available"""
        try:
            # Try to check for hardware acceleration capabilities
            result = subprocess.run(
                ['vainfo'], 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                text=True
            )
            return result.returncode == 0 and 'VAEntrypointVLD' in result.stdout
        except:
            return False

    def _flush_network_buffer(self):
        """Attempt to flush network buffers for the camera IP"""
        try:
            # Extract IP from RTSP URL
            import re
            ip_match = re.search(r'@?([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)', self.rtsp_url)
            if ip_match:
                ip = ip_match.group(1)
                # Attempt to flush network buffers
                self.get_logger().info(f"Flushing network buffers for {ip}")
                os.system(f"ping -c 1 {ip} > /dev/null")  # Wake up the connection
                os.system(f"sudo ip neigh flush {ip} > /dev/null 2>&1")  # Flush ARP cache
            else:
                self.get_logger().warn("Could not extract IP from RTSP URL for buffer flushing")
        except Exception as e:
            self.get_logger().error(f"Error flushing network buffer: {str(e)}")

    def start_capture(self):
        if not self.is_running:
            self.is_running = True
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            self.get_logger().info("Capture thread started")

    def get_pipeline_string(self, pipeline_type: int) -> str:
        """Get the GStreamer pipeline string based on the pipeline type."""
        # Increase GStreamer debug level for more verbose output
        os.environ["GST_DEBUG"] = "3,rtspsrc:5,rtph265depay:5,h265parse:5,avdec_h265:5"
        
        # Create a simpler pipeline specifically for this camera
        return (
            f"rtspsrc location={self.rtsp_url} protocols=tcp latency=0 ! "
            f"rtph265depay ! h265parse ! avdec_h265 ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink drop=true sync=false"
        )

    def capture_frames(self):
        """Capture frames from the RTSP stream and publish them."""
        consecutive_failures = 0
        max_consecutive_failures = 5
        reconnect_delay = 1.0  # seconds
        
        while rclpy.ok():
            try:
                # Set GStreamer debug level for more verbose output
                os.environ["GST_DEBUG"] = "3,rtspsrc:5,rtph265depay:5,h265parse:5,avdec_h265:5"
                
                # Create pipeline
                pipeline_str = self.get_pipeline_string(self.pipeline_type)
                self.get_logger().warn(f"Attempting to open RTSP stream at {self.rtsp_url}")
                self.get_logger().warn(f"Pipeline: {pipeline_str}")
                
                # Open stream with timeout
                cap = cv2.VideoCapture(pipeline_str, cv2.CAP_GSTREAMER)
                if not cap.isOpened():
                    raise RuntimeError(f"Failed to open RTSP stream at {self.rtsp_url}")
                
                # Test frame read
                ret, frame = cap.read()
                if not ret or frame is None:
                    raise RuntimeError("Failed to read initial frame from stream")
                
                self.get_logger().info(f"Successfully connected to RTSP stream at {self.rtsp_url}")
                consecutive_failures = 0  # Reset failure counter on successful connection
                
                # Main capture loop
                while rclpy.ok():
                    start_time = time.time()
                    
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        self.get_logger().warn("Failed to read frame from stream")
                        break
                    
                    # Convert frame to ROS message
                    try:
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher.publish(msg)
                        self.frame_count += 1
                        
                        # Calculate and log FPS
                        frame_time = time.time() - start_time
                        self.fps = 1.0 / frame_time if frame_time > 0 else 0
                        
                        if self.fps < 5.0:  # Only warn for very low FPS
                            self.get_logger().warn(f"Low FPS detected: {self.fps:.2f}")
                        
                    except Exception as e:
                        self.get_logger().warn(f"Error converting/publishing frame: {str(e)}")
                
                # If we break out of the inner loop, release the capture
                cap.release()
                
            except Exception as e:
                consecutive_failures += 1
                self.get_logger().warn(f"Error in capture loop (attempt {consecutive_failures}/{max_consecutive_failures}): {str(e)}")
                
                # Clean up resources
                if 'cap' in locals() and cap is not None:
                    cap.release()
                
                if consecutive_failures >= max_consecutive_failures:
                    self.get_logger().error(f"Too many consecutive failures ({consecutive_failures}), giving up")
                    break
                    
                # Wait before retrying
                time.sleep(reconnect_delay)

    def timer_callback(self):
        # Display frame rate and other diagnostics
        current_time = time.time()
        elapsed = current_time - self.last_log_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            self.last_fps = fps
            self.get_logger().info(f"Publishing at {fps:.2f} FPS")
            
            # Reset counters
            self.frame_count = 0
            self.last_log_time = current_time

    def destroy_node(self):
        self.get_logger().info("Shutting down RTSP Stream Node")
        self.is_running = False
        
        # Wait for capture thread to finish
        if self.capture_thread is not None and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        
        # Make sure to release the capture if it's still open
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            
        super().destroy_node()


def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create node
    node = RtspStreamNode()
    
    # Setup clean shutdown
    def signal_handler(sig, frame):
        node.get_logger().info("Received interrupt signal, shutting down...")
        node.destroy_node()
        # Don't call rclpy.shutdown() here, it will be called below
        sys.exit(0)
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This won't be called with our signal handler, but keep as a fallback
        node.get_logger().info("Keyboard interrupt received")
    finally:
        # Clean up node resources
        node.destroy_node()
        # Shutdown ROS
        rclpy.shutdown()


if __name__ == '__main__':
    main() 