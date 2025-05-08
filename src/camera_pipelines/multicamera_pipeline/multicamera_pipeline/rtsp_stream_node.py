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
        # Redirect stdout and stderr before anything else to catch all warnings
        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        self._stdout_devnull = None
        self._stderr_devnull = None
        
        # Set extremely aggressive environment variables for OpenCV and GStreamer
        os.environ["GST_DEBUG"] = "0"          # Disable GStreamer debug output completely
        os.environ["GST_SILENT"] = "1"         # Silence GStreamer messages
        os.environ["GST_DEBUG_NO_COLOR"] = "1" # Disable colored output
        os.environ["OPENCV_LOG_LEVEL"] = "0"   # Suppress OpenCV warnings
        
        # Initialize the ROS node
        super().__init__('rtsp_stream_node')
        
        # Declare and get only necessary parameters
        self.declare_parameter('rtsp_url', 'rtsp://192.168.2.250:554/stream')
        self.declare_parameter('frame_rate', 20.0)  # Hz, updated to match camera specs
        self.declare_parameter('image_width', 800)  # Match actual stream resolution
        self.declare_parameter('image_height', 448)  # Match actual stream resolution
        self.declare_parameter('topic_name', 'camera/image_raw')
        self.declare_parameter('pipeline_type', 4)  # Use TCP pipeline by default
        self.declare_parameter('silent_mode', True)  # Completely silent mode by default
        
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.pipeline_type = self.get_parameter('pipeline_type').get_parameter_value().integer_value
        self.silent_mode = self.get_parameter('silent_mode').get_parameter_value().bool_value
        
        # Redirect stdout and stderr if silent mode is enabled
        if self.silent_mode:
            # Set logging level to FATAL (only show critical errors that prevent operation)
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
            # Completely redirect stdout and stderr
            self._redirect_all_output()
        else:
            # Still use ERROR level to reduce verbosity but show problems
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        
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
        
        # Check for hardware acceleration - do silently
        self.has_vaapi = self._check_for_vaapi()
        
        # Create a timer for diagnostics (but don't log frequently)
        self.timer = self.create_timer(30.0, self.timer_callback)
        
        # Performance metrics
        self.frame_count = 0
        self.last_log_time = time.time()
        self.last_fps = 0.0
        
        # Video capture object
        self.cap = None
        
        # Flush any buffered frames on the network interface for this camera
        self._flush_network_buffer()
        
        # Only report that we're starting if not in silent mode
        if not self.silent_mode:
            self.get_logger().info(f"Starting RTSP stream from {self.rtsp_url} to {self.topic_name}")
        
        # Start capturing frames
        self.start_capture()

    def _redirect_all_output(self):
        """Completely redirect both stdout and stderr to /dev/null"""
        try:
            self._stdout_devnull = open(os.devnull, 'w')
            self._stderr_devnull = open(os.devnull, 'w')
            sys.stdout = self._stdout_devnull
            sys.stderr = self._stderr_devnull
        except Exception:
            # If this fails, restore original handles
            sys.stdout = self._original_stdout
            sys.stderr = self._original_stderr

    def _check_for_vaapi(self):
        """Check if VAAPI hardware acceleration is available"""
        try:
            # Silently check for hardware acceleration capabilities 
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
                # Attempt to flush network buffers silently
                os.system(f"ping -c 1 {ip} > /dev/null 2>&1")  # Wake up the connection
                os.system(f"sudo ip neigh flush {ip} > /dev/null 2>&1")  # Flush ARP cache
        except Exception as e:
            if not self.silent_mode:
                self.get_logger().error(f"Error flushing network buffer: {str(e)}")

    def start_capture(self):
        if not self.is_running:
            self.is_running = True
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.daemon = True
            self.capture_thread.start()

    def get_pipeline_string(self, pipeline_type: int) -> str:
        """Get the GStreamer pipeline string based on the pipeline type."""
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
                # Create pipeline
                pipeline_str = self.get_pipeline_string(self.pipeline_type)
                
                # Open stream with timeout - suppress warnings with environment variables
                cap = cv2.VideoCapture(pipeline_str, cv2.CAP_GSTREAMER)
                if not cap.isOpened():
                    raise RuntimeError(f"Failed to open RTSP stream at {self.rtsp_url}")
                
                # Test frame read
                ret, frame = cap.read()
                if not ret or frame is None:
                    raise RuntimeError("Failed to read initial frame from stream")
                
                # Only report once we're successfully connected and received the first frame
                if not self.silent_mode:
                    self.get_logger().info(f"Successfully connected to {self.rtsp_url}, now streaming to {self.topic_name}")
                consecutive_failures = 0  # Reset failure counter on successful connection
                
                # Main capture loop - runs silently unless errors occur
                frame_counter = 0
                while rclpy.ok():
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        if not self.silent_mode:
                            self.get_logger().error("Failed to read frame from stream")
                        break
                    
                    # Convert frame to ROS message
                    try:
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher.publish(msg)
                        self.frame_count += 1
                        frame_counter += 1
                    except Exception as e:
                        if not self.silent_mode:
                            self.get_logger().error(f"Error converting/publishing frame: {str(e)}")
                
                # If we break out of the inner loop, release the capture
                cap.release()
                
            except Exception as e:
                consecutive_failures += 1
                if not self.silent_mode:
                    self.get_logger().error(f"Error in capture loop (attempt {consecutive_failures}/{max_consecutive_failures}): {str(e)}")
                
                # Clean up resources
                if 'cap' in locals() and cap is not None:
                    cap.release()
                
                if consecutive_failures >= max_consecutive_failures:
                    if not self.silent_mode:
                        self.get_logger().error(f"Too many consecutive failures ({consecutive_failures}), giving up")
                    break
                    
                # Wait before retrying
                time.sleep(reconnect_delay)

    def timer_callback(self):
        # Only log FPS if it's abnormally low to indicate a potential problem
        current_time = time.time()
        elapsed = current_time - self.last_log_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            self.last_fps = fps
            
            # Only log if FPS is very low (potential problem) and not in silent mode
            if fps < 5.0 and not self.silent_mode:
                self.get_logger().warn(f"Low frame rate detected: {fps:.2f} FPS")
            
            # Reset counters
            self.frame_count = 0
            self.last_log_time = current_time

    def destroy_node(self):
        if not self.silent_mode:
            self.get_logger().info(f"Shutting down RTSP stream from {self.rtsp_url}")
        self.is_running = False
        
        # Wait for capture thread to finish
        if self.capture_thread is not None and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        
        # Make sure to release the capture if it's still open
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            
        # Restore stdout and stderr if we redirected them
        if self._stdout_devnull is not None:
            sys.stdout = self._original_stdout
            self._stdout_devnull.close()
            
        if self._stderr_devnull is not None:
            sys.stderr = self._original_stderr
            self._stderr_devnull.close()
            
        super().destroy_node()


def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create node
    node = RtspStreamNode()
    
    # Setup clean shutdown
    def signal_handler(sig, frame):
        node.destroy_node()
        # Don't call rclpy.shutdown() here, it will be called below
        sys.exit(0)
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This won't be called with our signal handler, but keep as a fallback
        pass
    finally:
        # Clean up node resources
        node.destroy_node()
        # Shutdown ROS
        rclpy.shutdown()


if __name__ == '__main__':
    main() 