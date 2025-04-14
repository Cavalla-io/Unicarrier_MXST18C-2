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


class RtspStreamNode(Node):
    def __init__(self):
        super().__init__('rtsp_stream_node')
        
        # Declare and get only necessary parameters
        self.declare_parameter('rtsp_url', 'rtsp://192.168.2.250:554/stream')
        self.declare_parameter('frame_rate', 30.0)  # Hz
        self.declare_parameter('image_width', 640)  # default width
        self.declare_parameter('image_height', 480)  # default height
        self.declare_parameter('topic_name', 'camera/image_raw')
        self.declare_parameter('pipeline_type', 1)  # Select which pipeline to use (1-4)
        
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.pipeline_type = self.get_parameter('pipeline_type').get_parameter_value().integer_value
        
        # Create publisher with minimal queue size for lowest latency
        self.publisher = self.create_publisher(Image, self.topic_name, 1)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize capture thread
        self.is_running = False
        self.capture_thread = None
        
        # Check for hardware acceleration
        self.has_vaapi = self._check_for_vaapi()
        self.get_logger().info(f"Hardware acceleration (VAAPI): {'Available' if self.has_vaapi else 'Not available'}")
        
        # Create a timer for diagnostics
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        # Performance metrics
        self.frame_count = 0
        self.last_log_time = time.time()
        
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
            ip_match = re.search(r'//([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)', self.rtsp_url)
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

    def get_pipeline_string(self):
        """Get the appropriate GStreamer pipeline based on pipeline_type"""
        # Ultra-low latency with UDP
        if self.pipeline_type == 1:
            return (
                f"rtspsrc location={self.rtsp_url} latency=0 buffer-mode=none "
                f"transport=udp drop-on-latency=true ! queue max-size-buffers=0 max-size-time=0 ! "
                f"rtph264depay ! h264parse ! "
                f"avdec_h264 max-threads=4 ! videoconvert ! video/x-raw,format=BGR ! "
                f"appsink max-buffers=1 drop=true sync=false"
            )
        # Alternative pipeline optimized for different cameras
        elif self.pipeline_type == 2:
            return (
                f"rtspsrc location={self.rtsp_url} latency=0 "
                f"! rtph264depay ! h264parse ! "
                f"decodebin ! videoconvert ! video/x-raw,format=BGR ! "
                f"appsink max-buffers=1 drop=true sync=false"
            )
        # Try hardware acceleration if available
        elif self.pipeline_type == 3 and self.has_vaapi:
            return (
                f"rtspsrc location={self.rtsp_url} latency=0 buffer-mode=none "
                f"! rtph264depay ! h264parse ! "
                f"vaapidecode ! vaapipostproc format=bgr ! "
                f"appsink max-buffers=1 drop=true sync=false"
            )
        # Direct TCP connection, bypassing some RTSP overhead
        elif self.pipeline_type == 4:
            return (
                f"rtspsrc location={self.rtsp_url} latency=0 buffer-mode=none "
                f"protocols=tcp transport=tcp use-buffering=false ! "
                f"rtph264depay ! h264parse ! "
                f"avdec_h264 max-threads=4 ! videoconvert ! video/x-raw,format=BGR ! "
                f"appsink max-buffers=1 drop=true sync=false"
            )
        # Fallback to default pipeline
        else:
            return (
                f"rtspsrc location={self.rtsp_url} latency=0 buffer-mode=auto "
                f"transport=udp drop-on-latency=true ! rtph264depay ! h264parse ! "
                f"avdec_h264 max-threads=4 ! videoconvert ! video/x-raw,format=BGR ! "
                f"appsink max-buffers=1 drop=true sync=false"
            )

    def capture_frames(self):
        # Get the pipeline based on selected type
        pipeline = self.get_pipeline_string()
        self.get_logger().info(f"Using GStreamer pipeline: {pipeline}")
        
        # Set environment variables to influence GStreamer behavior
        os.environ["GST_DEBUG"] = "3"  # Set GStreamer debug level
        os.environ["GST_BUFFER_POOL_MAX_SIZE"] = "1"  # Limit buffer pool size
        
        # Open the video capture
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        # Check if camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open RTSP stream at {self.rtsp_url}")
            self.is_running = False
            return
        
        # Try to set additional properties to reduce latency
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Only buffer 1 frame
        
        last_frame_time = time.time()
        last_reconnect_time = time.time()
        
        while self.is_running and rclpy.ok():
            # Capture frame with timeout
            start_read = time.time()
            ret, frame = self.cap.read()
            read_time = time.time() - start_read
            
            current_time = time.time()
            
            # If read took too long, log it
            if read_time > 0.1:
                self.get_logger().debug(f"Frame read took {read_time:.2f}s, potentially adding latency")
            
            if not ret:
                self.get_logger().warn("Failed to get frame from RTSP stream")
                
                # Don't try to reconnect too frequently
                if current_time - last_reconnect_time > 2.0:
                    if self.cap is not None:
                        self.cap.release()
                        self.cap = None
                    
                    # Flush network buffers before reconnecting
                    self._flush_network_buffer()
                    
                    time.sleep(0.2)  # Shorter wait before trying to reconnect
                    
                    # Recreate pipeline with same settings
                    pipeline = self.get_pipeline_string()
                    self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                    last_reconnect_time = current_time
                
                continue
            
            # Convert to ROS Image message and publish immediately
            try:
                # Start conversion timing
                start_convert = time.time()
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header = Header()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_frame"
                
                # Publish the image
                self.publisher.publish(img_msg)
                
                convert_time = time.time() - start_convert
                if convert_time > 0.05:
                    self.get_logger().debug(f"Image conversion took {convert_time:.2f}s")
                
                self.frame_count += 1
                last_frame_time = current_time
            except Exception as e:
                self.get_logger().error(f"Error converting/publishing frame: {str(e)}")
        
        # Clean up
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def timer_callback(self):
        # Display frame rate and other diagnostics
        current_time = time.time()
        elapsed = current_time - self.last_log_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
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