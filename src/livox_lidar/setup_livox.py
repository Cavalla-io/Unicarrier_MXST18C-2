import rclpy
from rclpy.node import Node
from livox_ros_driver2.srv import SetExtrinsicParameters, SetImuParameters
import numpy as np
from typing import Dict, List

# Define LiDAR positions and orientations
LIDAR_CONFIG = {
    'front_left': {
        'position': np.array([2.0, -0.5, 1.5]),  # meters from center
        'rotation': np.array([0, 0, 0]),        # radians
        'ip_address': '192.168.1.101'
    },
    'front_right': {
        'position': np.array([2.0, 0.5, 1.5]),
        'rotation': np.array([0, 0, 0]),
        'ip_address': '192.168.1.102'
    },
    'rear': {
        'position': np.array([-2.0, 0, 1.5]),
        'rotation': np.array([0, 0, np.pi]),  # 180 degrees
        'ip_address': '192.168.1.103'
    }
}

class LivoxSetupNode(Node):
    def __init__(self):
        super().__init__('livox_setup_node')
        self.clients = {}
        self.setup_clients()
        
    def setup_clients(self):
        # Create service clients for each LiDAR
        for lidar_name in LIDAR_CONFIG.keys():
            self.clients[lidar_name] = {
                'extrinsic': self.create_client(
                    SetExtrinsicParameters,
                    f'livox/{lidar_name}/set_extrinsic_parameters'
                ),
                'imu': self.create_client(
                    SetImuParameters,
                    f'livox/{lidar_name}/set_imu_parameters'
                )
            }
            
            # Wait for services to be available
            while not self.clients[lidar_name]['extrinsic'].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {lidar_name} extrinsic service...')
            
            while not self.clients[lidar_name]['imu'].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {lidar_name} IMU service...')
                
    def configure_lidar(self, lidar_name: str):
        config = LIDAR_CONFIG[lidar_name]
        
        # Set extrinsic parameters
        extrinsic_req = SetExtrinsicParameters.Request()
        extrinsic_req.position = config['position'].tolist()
        extrinsic_req.rotation = config['rotation'].tolist()
        
        try:
            extrinsic_future = self.clients[lidar_name]['extrinsic'].call_async(extrinsic_req)
            rclpy.spin_until_future_complete(self, extrinsic_future)
            
            if extrinsic_future.result() is not None:
                self.get_logger().info(f'Successfully set extrinsics for {lidar_name}')
            else:
                self.get_logger().error(f'Failed to set extrinsics for {lidar_name}')
                
            # Set IMU parameters
            imu_req = SetImuParameters.Request()
            imu_req.enable = True
            imu_req.frequency = 100.0  # Hz
            
            imu_future = self.clients[lidar_name]['imu'].call_async(imu_req)
            rclpy.spin_until_future_complete(self, imu_future)
            
            if imu_future.result() is not None:
                self.get_logger().info(f'Successfully configured IMU for {lidar_name}')
            else:
                self.get_logger().error(f'Failed to configure IMU for {lidar_name}')
                
        except Exception as e:
            self.get_logger().error(f'Error configuring {lidar_name}: {str(e)}')
            
    def configure_all_lidars(self):
        for lidar_name in LIDAR_CONFIG.keys():
            self.configure_lidar(lidar_name)
            
    def shutdown(self):
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LivoxSetupNode()
    try:
        node.configure_all_lidars()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
