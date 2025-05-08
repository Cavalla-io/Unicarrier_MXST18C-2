#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import numpy as np
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import threading
import sys

# Import psutil for system monitoring
try:
    import psutil
    HAVE_PSUTIL = True
except ImportError:
    HAVE_PSUTIL = False


class CameraPerformanceMonitor(Node):
    """Monitor camera streams for performance metrics like latency, drops, and freezes."""

    def __init__(self):
        super().__init__('camera_performance_monitor')
        
        # Configuration parameters
        self.declare_parameter('camera_topics', [
            'front_camera/rgb/image_raw',
            'fork_camera/rgb/image_raw',
            'rtsp_camera1/image_raw',
            'rtsp_camera2/image_raw'
        ])
        self.declare_parameter('display_debug_image', False)
        self.declare_parameter('log_to_file', True)
        self.declare_parameter('freeze_detection_threshold', 0.98)  # Image similarity threshold to detect freezes
        self.declare_parameter('monitor_period_sec', 5.0)  # How often to update metrics
        self.declare_parameter('run_duration_sec', 30.0)  # Default 30 second monitoring duration
        self.declare_parameter('silent_mode', False)  # Allow terminal output by default
        self.declare_parameter('monitor_system_resources', True)  # Monitor CPU and memory by default
        
        # Get parameters
        self.camera_topics = self.get_parameter('camera_topics').value
        self.display_debug = self.get_parameter('display_debug_image').value
        self.log_to_file = self.get_parameter('log_to_file').value
        self.freeze_threshold = self.get_parameter('freeze_detection_threshold').value
        self.monitor_period = self.get_parameter('monitor_period_sec').value
        self.run_duration = self.get_parameter('run_duration_sec').value
        self.silent_mode = self.get_parameter('silent_mode').value
        self.monitor_resources = self.get_parameter('monitor_system_resources').value and HAVE_PSUTIL
        
        # System resource metrics
        self.system_metrics = {
            'cpu_percent': [],
            'memory_percent': [],
            'timestamps': []
        }
        
        # Check if psutil is available and warn if we can't monitor system resources
        if self.monitor_resources:
            self.get_logger().info("System resource monitoring enabled (CPU and memory)")
        elif self.get_parameter('monitor_system_resources').value and not HAVE_PSUTIL:
            self.get_logger().warning("Cannot monitor system resources - psutil not available")
            self.get_logger().warning("Install psutil with: pip install psutil")
        
        # Set up QoS - we want to detect drops so need appropriate QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize metrics dictionaries
        self.metrics = {}
        self.cv_bridge = CvBridge()
        
        # Print clear header to terminal
        print("\n" + "="*80)
        print(f"CAMERA PERFORMANCE TEST - Running for {self.run_duration:.1f} seconds")
        print("="*80)
        print("Monitoring topics:")
        
        # Create subscribers for each camera topic
        self.subscribers = {}
        for topic in self.camera_topics:
            print(f"  - {topic}")
            self.subscribers[topic] = self.create_subscription(
                Image, 
                topic, 
                lambda msg, topic=topic: self.image_callback(msg, topic),
                qos
            )
            
            # Initialize metrics for this topic
            self.metrics[topic] = {
                'frame_count': 0,
                'last_frame_time': None,
                'last_header_stamp': None,
                'latencies': [],
                'intervals': [],
                'start_time': time.time(),
                'last_image': None,
                'freeze_count': 0,
                'dropped_frames': 0,
                'last_seq': -1
            }
        
        # Create timer for periodic reporting
        self.timer = self.create_timer(self.monitor_period, self.update_metrics)
        
        # Create timer for resource monitoring (more frequent sampling)
        if self.monitor_resources:
            self.resource_timer = self.create_timer(1.0, self.sample_system_resources)
        
        # File logging
        if self.log_to_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file = open(f"camera_metrics_{timestamp}.csv", "w")
            self.log_file.write("timestamp,topic,frame_rate,avg_latency_ms,max_latency_ms,freeze_count,drop_rate\n")
            
            # Create separate resource log if monitoring resources
            if self.monitor_resources:
                self.resource_file = open(f"system_resources_{timestamp}.csv", "w")
                self.resource_file.write("timestamp,cpu_percent,memory_percent\n")
            
        # Setup countdown and auto-shutdown
        self.start_time = time.time()
        # Use shorter countdown interval for more visible feedback
        self.shutdown_timer = self.create_timer(1.0, self.countdown_tick)
        
        self.get_logger().info(f"Camera performance monitor started - will run for {self.run_duration:.1f} seconds")
        print(f"\nPerformance test starting...")
        print(f"Press Ctrl+C to stop early and view results\n")
    
    def sample_system_resources(self):
        """Sample CPU and memory usage"""
        try:
            cpu_percent = psutil.cpu_percent(interval=None)  # Non-blocking call
            memory_percent = psutil.virtual_memory().percent
            
            self.system_metrics['cpu_percent'].append(cpu_percent)
            self.system_metrics['memory_percent'].append(memory_percent)
            self.system_metrics['timestamps'].append(time.time())
            
            # Log resource data to file
            if self.log_to_file and hasattr(self, 'resource_file'):
                self.resource_file.write(f"{time.time()},{cpu_percent},{memory_percent}\n")
                self.resource_file.flush()
                
        except Exception as e:
            self.get_logger().warning(f"Error sampling system resources: {e}")
    
    def countdown_tick(self):
        """Display countdown and shut down when time is up"""
        elapsed = time.time() - self.start_time
        remaining = max(0, self.run_duration - elapsed)
        
        # Show countdown every second
        if int(remaining) % 5 == 0 or remaining <= 5:
            # Use direct print instead of ROS logger for terminal visibility
            if self.monitor_resources:
                # Include current CPU/memory usage in countdown if monitoring resources
                cpu = self.system_metrics['cpu_percent'][-1] if self.system_metrics['cpu_percent'] else 0
                mem = self.system_metrics['memory_percent'][-1] if self.system_metrics['memory_percent'] else 0
                print(f"Monitoring cameras: {int(remaining)}s left | CPU: {cpu:.1f}% | Mem: {mem:.1f}%   ", end="\r", flush=True)
            else:
                print(f"Monitoring cameras: {int(remaining)} seconds remaining   ", end="\r", flush=True)
        
        # Time's up - report and shut down
        if elapsed >= self.run_duration:
            print("\nTest complete! Generating report...                    ")
            self.shutdown_timer.cancel()
            self.timer.cancel()
            if self.monitor_resources and hasattr(self, 'resource_timer'):
                self.resource_timer.cancel()
            self.report_final_metrics()
            threading.Timer(2.0, self.shutdown).start()  # Give time for metrics to be printed
    
    def shutdown(self):
        """Clean shutdown after reporting metrics"""
        if self.display_debug:
            cv2.destroyAllWindows()
        if self.log_to_file and hasattr(self, 'log_file'):
            self.log_file.close()
        if self.log_to_file and hasattr(self, 'resource_file'):
            self.resource_file.close()
        self.get_logger().info("Performance monitoring completed")
        sys.exit(0)
    
    def image_callback(self, msg, topic):
        """Process each incoming image frame and update metrics."""
        now = time.time()
        metrics = self.metrics[topic]
        
        # Extract timestamp from header for latency calculation
        if msg.header.stamp.sec > 0:  # Only if timestamp is valid
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            transport_latency = now - msg_time
            metrics['latencies'].append(transport_latency * 1000)  # Store in ms
        
        # Calculate inter-frame interval
        if metrics['last_frame_time'] is not None:
            interval = now - metrics['last_frame_time']
            metrics['intervals'].append(interval)
            
            # Detect potential drops (if interval is significantly larger than average)
            if len(metrics['intervals']) > 10:
                avg_interval = sum(metrics['intervals'][-10:]) / min(10, len(metrics['intervals']))
                if interval > 2.5 * avg_interval and avg_interval > 0:
                    metrics['dropped_frames'] += 1
        
        # Freeze detection
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if metrics['last_image'] is not None:
                # Calculate image similarity using normalized correlation
                # Downscale for faster processing
                small1 = cv2.resize(metrics['last_image'], (160, 120))
                small2 = cv2.resize(cv_image, (160, 120))
                gray1 = cv2.cvtColor(small1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(small2, cv2.COLOR_BGR2GRAY)
                
                # Calculate correlation coefficient
                correlation = cv2.matchTemplate(gray1, gray2, cv2.TM_CCORR_NORMED)[0][0]
                
                # If images are too similar, it might be a freeze
                if correlation > self.freeze_threshold:
                    metrics['freeze_count'] += 1
            
            # Store current image for next comparison
            metrics['last_image'] = cv_image
            
            # Debug visualization
            if self.display_debug:
                cv2.imshow(f"Camera Feed: {topic}", cv_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
        
        # Update frame metrics
        metrics['frame_count'] += 1
        metrics['last_frame_time'] = now
        metrics['last_header_stamp'] = msg.header.stamp
    
    def update_metrics(self):
        """Update metrics and show live stats periodically"""
        now = time.time()
        elapsed = now - self.start_time
        
        # Only log to file, don't print to console in silent mode
        if not self.silent_mode and elapsed > 5.0:  # Show first update after 5 seconds
            # Clear line and show some basic stats
            print("                                                                            ", end="\r")
            fps_info = " | ".join([f"{topic.split('/')[-2]}: {len(metrics['intervals'])>0 and 1.0/sum(metrics['intervals'][-10:])*min(10, len(metrics['intervals'])):.1f} fps" 
                                for topic, metrics in self.metrics.items() if metrics['frame_count'] > 0])
            
            # Add resource info if available
            if self.monitor_resources and self.system_metrics['cpu_percent']:
                cpu = self.system_metrics['cpu_percent'][-1]
                mem = self.system_metrics['memory_percent'][-1]
                print(f"Current FPS: {fps_info} | CPU: {cpu:.1f}% | Mem: {mem:.1f}%", end="\r", flush=True)
            else:
                print(f"Current FPS: {fps_info}", end="\r", flush=True)
        
        # Log metrics to file
        for topic, metrics in self.metrics.items():
            if metrics['frame_count'] == 0:
                continue
            
            # Calculate metrics
            duration = now - metrics['start_time']
            frame_rate = metrics['frame_count'] / duration
            
            avg_latency = 0
            max_latency = 0
            if metrics['latencies']:
                avg_latency = sum(metrics['latencies']) / len(metrics['latencies']) 
                max_latency = max(metrics['latencies'])
            
            drop_rate = 0
            if metrics['frame_count'] > 0:
                drop_rate = metrics['dropped_frames'] / metrics['frame_count']
            
            # Log to file
            if self.log_to_file:
                self.log_file.write(f"{now},{topic},{frame_rate:.2f},{avg_latency:.2f},{max_latency:.2f},"
                                   f"{metrics['freeze_count']},{drop_rate:.4f}\n")
                self.log_file.flush()
    
    def report_final_metrics(self):
        """Report complete performance metrics at the end of the run."""
        now = time.time()
        
        # Print header
        print("\n" + "="*80)
        print(f"CAMERA PERFORMANCE REPORT - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)
        
        print(f"Total monitoring duration: {now - self.start_time:.1f} seconds\n")
        
        # System resource metrics
        if self.monitor_resources and self.system_metrics['cpu_percent']:
            avg_cpu = sum(self.system_metrics['cpu_percent']) / len(self.system_metrics['cpu_percent'])
            max_cpu = max(self.system_metrics['cpu_percent'])
            avg_mem = sum(self.system_metrics['memory_percent']) / len(self.system_metrics['memory_percent'])
            max_mem = max(self.system_metrics['memory_percent'])
            
            print("--- System Resource Usage ---")
            print(f"  Average CPU Usage:   {avg_cpu:.1f}%")
            print(f"  Maximum CPU Usage:   {max_cpu:.1f}%")
            print(f"  Average Memory:      {avg_mem:.1f}%")
            print(f"  Maximum Memory:      {max_mem:.1f}%")
            print()
        
        # Track if any camera had zero frames
        zero_frame_cameras = []
        
        for topic, metrics in self.metrics.items():
            if metrics['frame_count'] == 0:
                zero_frame_cameras.append(topic)
                continue
            
            # Calculate metrics
            duration = now - metrics['start_time']
            frame_rate = metrics['frame_count'] / duration
            
            avg_latency = 0
            max_latency = 0
            min_latency = 0
            if metrics['latencies']:
                avg_latency = sum(metrics['latencies']) / len(metrics['latencies']) 
                max_latency = max(metrics['latencies'])
                min_latency = min(metrics['latencies'])
            
            avg_interval = 0
            max_interval = 0
            if metrics['intervals']:
                avg_interval = sum(metrics['intervals']) / len(metrics['intervals']) 
                max_interval = max(metrics['intervals'])
                expected_interval = 1.0 / frame_rate if frame_rate > 0 else 0
            
            drop_rate = 0
            if metrics['frame_count'] > 0:
                drop_rate = metrics['dropped_frames'] / metrics['frame_count']
            
            # Formatted output for each camera
            print(f"--- {topic} Performance ---")
            print(f"  Frame Count:        {metrics['frame_count']} frames")
            print(f"  Frame Rate:         {frame_rate:.2f} fps")
            print(f"  Average Latency:    {avg_latency:.2f} ms")
            print(f"  Min/Max Latency:    {min_latency:.2f} / {max_latency:.2f} ms")
            print(f"  Frame Interval:     {(avg_interval*1000):.2f} ms (expected: {(expected_interval*1000):.2f} ms)")
            print(f"  Maximum Interval:   {(max_interval*1000):.2f} ms")
            print(f"  Detected Freezes:   {metrics['freeze_count']}")
            print(f"  Estimated Drops:    {metrics['dropped_frames']} ({drop_rate*100:.2f}%)")
            print()
        
        # Report any cameras with zero frames
        if zero_frame_cameras:
            print("\nWARNING: The following cameras received no frames during monitoring:")
            for topic in zero_frame_cameras:
                print(f"  - {topic}")
        
        # Performance analysis and recommendations
        print("\n--- Performance Analysis ---")
        if self.monitor_resources and avg_cpu > 75:
            print("⚠️ HIGH CPU USAGE: The system appears CPU-constrained during operation")
            if avg_cpu > 90:
                print("   RECOMMENDATION: Reduce the number of cameras or lower camera resolution/FPS")
        
        if self.monitor_resources and avg_mem > 80:
            print("⚠️ HIGH MEMORY USAGE: The system is approaching memory limits")
            print("   RECOMMENDATION: Check for memory leaks or reduce frame buffer sizes")
        
        high_latency_cameras = [topic for topic, metrics in self.metrics.items() 
                               if metrics['latencies'] and max(metrics['latencies']) > 500]
        if high_latency_cameras:
            print("⚠️ HIGH LATENCY detected on these cameras:")
            for topic in high_latency_cameras:
                print(f"   - {topic}")
            print("   RECOMMENDATION: Check network conditions and camera settings")
        
        high_drop_cameras = [topic for topic, metrics in self.metrics.items() 
                            if metrics['frame_count'] > 0 and 
                            metrics['dropped_frames'] / metrics['frame_count'] > 0.1]
        if high_drop_cameras:
            print("⚠️ HIGH DROP RATE detected on these cameras:")
            for topic in high_drop_cameras:
                print(f"   - {topic}")
            print("   RECOMMENDATION: Use TCP transport for RTSP streams or check network bandwidth")
            
        high_freeze_cameras = [topic for topic, metrics in self.metrics.items() 
                              if metrics['freeze_count'] > 5]
        if high_freeze_cameras:
            print("⚠️ FREQUENT FREEZES detected on these cameras:")
            for topic in high_freeze_cameras:
                print(f"   - {topic}")
            print("   RECOMMENDATION: Check if camera settings match system capabilities")
            
        # Log file info
        print("\n" + "="*80)
        if self.log_to_file:
            print(f"Detailed camera metrics saved to: {self.log_file.name}")
            if self.monitor_resources:
                print(f"System resource data saved to:  {self.resource_file.name}")
    
    def destroy_node(self):
        """Clean up resources when node is shut down."""
        if self.display_debug:
            cv2.destroyAllWindows()
        if self.log_to_file and hasattr(self, 'log_file'):
            self.log_file.close()
        if self.log_to_file and hasattr(self, 'resource_file'):
            self.resource_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    monitor = CameraPerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        # Print stats even if interrupted
        print("\nTest interrupted! Generating report...                ")
        monitor.report_final_metrics()
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 