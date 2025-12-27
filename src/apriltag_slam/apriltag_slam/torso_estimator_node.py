#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
import numpy as np

class TorsoEstimatorNode(Node):
    def __init__(self):
        super().__init__('torso_estimator')
        
        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/apriltag_slam/camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.get_logger().info('Torso Estimator Node Started!')
    
    def imu_callback(self, msg: Imu):
        # Extract IMU data
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.get_logger().info(f'IMU: acc={acc}, gyro={gyro}', throttle_duration_sec=1.0)
    
    def camera_callback(self, msg: Image):
        self.get_logger().info(f'Camera: {msg.width}x{msg.height}', throttle_duration_sec=1.0)

def main():
    rclpy.init()
    node = TorsoEstimatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
