import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
from livox_ros_driver2.msg import CustomMsg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2, PointField
import struct
import ctypes
import threading
import time
from typing import Dict, List, Optional

# Define safety zones for each LiDAR
SAFETY_ZONES = {
    'front_left': {
        'min_distance': 1.0,  # meters
        'warning_distance': 2.0,
        'max_distance': 10.0,
        'angle_range': (-45, 45)  # degrees
    },
    'front_right': {
        'min_distance': 1.0,
        'warning_distance': 2.0,
        'max_distance': 10.0,
        'angle_range': (-45, 45)
    },
    'rear': {
        'min_distance': 1.0,
        'warning_distance': 2.0,
        'max_distance': 10.0,
        'angle_range': (-45, 45)
    }
}

class LivoxLidarNode(Node):
    def __init__(self):
        super().__init__('livox_lidar_node')
        
        # Initialize LiDARs
        self.lidars = {
            'front_left': None,
            'front_right': None,
            'rear': None
        }
        
        # Create publishers
        self.create_publishers()
        
        # Create subscribers
        self.create_subscribers()
        
        # Create safety monitor
        self.safety_monitor = LidarSafetyMonitor(self)
        
        # Create timer for periodic updates
        self.timer = self.create_timer(0.1, self.update)  # 10 Hz
        
        self.get_logger().info('Livox LiDAR node initialized')
        
    def create_publishers(self):
        # Point cloud publishers
        self.pc_publishers = {
            name: self.create_publisher(
                PointCloud2,
                f'lidar/{name}/point_cloud',
                self.get_qos_profile()
            ) for name in self.lidars.keys()
        }
        
        # Safety status publisher
        self.safety_publisher = self.create_publisher(
            BoolStamped,
            'lidar/safety_status',
            self.get_qos_profile()
        )
        
    def create_subscribers(self):
        # Subscribe to Livox data
        for lidar_name in self.lidars.keys():
            self.create_subscription(
                CustomMsg,
                f'livox/{lidar_name}/data',
                lambda msg, name=lidar_name: self.lidar_callback(msg, name),
                self.get_qos_profile()
            )
            
    def get_qos_profile(self):
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        return qos_profile
        
    def lidar_callback(self, msg: CustomMsg, lidar_name: str):
        try:
            # Process point cloud data
            point_cloud = self.process_point_cloud(msg)
            
            # Publish point cloud
            self.publish_point_cloud(lidar_name, point_cloud)
            
            # Update safety monitor
            self.safety_monitor.update(lidar_name, point_cloud)
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data from {lidar_name}: {str(e)}')
            
    def process_point_cloud(self, msg: CustomMsg) -> List[Point32]:
        points = []
        for point in msg.point:
            # Convert to Point32
            p = Point32()
            p.x = point.x
            p.y = point.y
            p.z = point.z
            points.append(p)
        return points
        
    def publish_point_cloud(self, lidar_name: str, points: List[Point32]):
        pc_msg = PointCloud2()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = f'lidar_{lidar_name}_frame'
        
        # Set point cloud fields
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.point_step = 12  # 3 floats (x,y,z) * 4 bytes each
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        
        # Convert points to byte array
        data = bytearray()
        for point in points:
            data.extend(struct.pack('fff', point.x, point.y, point.z))
        
        pc_msg.data = data
        
        self.pc_publishers[lidar_name].publish(pc_msg)
        
    def update(self):
        # Periodic safety check
        self.safety_monitor.check_safety()
        
    def shutdown(self):
        self.timer.cancel()
        self.destroy_node()
        
        # Clean up LiDAR connections
        for lidar in self.lidars.values():
            if lidar is not None:
                lidar.disconnect()

class LidarSafetyMonitor:
    def __init__(self, node: Node):
        self.node = node
        self.current_state = True  # True = safe, False = unsafe
        self.last_update = time.time()
        self.point_clouds = {name: None for name in SAFETY_ZONES.keys()}
        
    def update(self, lidar_name: str, point_cloud: List[Point32]):
        self.point_clouds[lidar_name] = point_cloud
        self.last_update = time.time()
        
    def check_safety(self):
        current_time = time.time()
        if current_time - self.last_update > 1.0:  # 1 second timeout
            self.current_state = False
            self.publish_safety_status()
            return
            
        for lidar_name, points in self.point_clouds.items():
            if points is None:
                continue
                
            # Get safety zone parameters
            zone = SAFETY_ZONES[lidar_name]
            min_dist = zone['min_distance']
            warning_dist = zone['warning_distance']
            max_dist = zone['max_distance']
            angle_range = zone['angle_range']
            
            # Convert angle range to radians
            angle_min = np.radians(angle_range[0])
            angle_max = np.radians(angle_range[1])
            
            # Check each point
            for point in points:
                # Calculate distance and angle
                distance = np.sqrt(point.x**2 + point.y**2)
                angle = np.arctan2(point.y, point.x)
                
                # Check if point is within safety zone
                if angle_min <= angle <= angle_max and distance < warning_dist:
                    self.current_state = False
                    self.publish_safety_status()
                    return
                    
        self.current_state = True
        self.publish_safety_status()
        
    def publish_safety_status(self):
        msg = BoolStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = self.current_state
        self.node.safety_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
