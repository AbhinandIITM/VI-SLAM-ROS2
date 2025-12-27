#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
import time

class RelativeTagMover(Node):
    def __init__(self):
        super().__init__('random_tag_mover')
        
        # --- 1. Service Client to Move Tag ---
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service...')
            
        # --- 2. Subscriber for Robot Ground Truth ---
        # We need this to know where the robot is, so we can put the tag in front of it.
        self.robot_pose = None
        self.create_subscription(Odometry, '/apriltag_slam/ground_truth/odom', self.odom_callback, 10)
        
        # --- 3. Timer Loop ---
        self.timer = self.create_timer(0.033, self.move_tag_relative) # 30 Hz
        self.start_time = time.time()
        self.get_logger().info("Relative Tag Mover Started: Tag will follow the robot.")

    def odom_callback(self, msg):
        # Store the latest robot pose (Position and Orientation)
        self.robot_pose = msg.pose.pose

    def move_tag_relative(self):
        if self.robot_pose is None:
            return # Wait until we know where the robot is

        t = time.time() - self.start_time
        
        # --- A. Define Tag Position in ROBOT BODY Frame ---
        # (X=Forward, Y=Left, Z=Up) relative to the robot's base
        
        # Depth: Keep it 1.3m to 1.7m in front of the robot
        local_x = 1.5 + 0.2 * math.sin(t * 0.5)
        
        # Lateral: Oscillate Left/Right +/- 0.3m
        local_y = 0.3 * math.sin(t * 0.8)
        
        # Height: Oscillate Up/Down (0.2m to 0.5m)
        local_z = 0.35 + 0.15 * math.cos(t * 0.6)
        
        local_point = np.array([local_x, local_y, local_z])

        # --- B. Get Robot's World Transform ---
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y
        rz = self.robot_pose.position.z
        
        # Robot Orientation (Quaternion)
        rq = [
            self.robot_pose.orientation.x,
            self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,
            self.robot_pose.orientation.w
        ]
        
        # --- C. Transform Local Point -> World Point ---
        # P_world = R_robot * P_local + T_robot
        rot_matrix = R.from_quat(rq)
        world_point = rot_matrix.apply(local_point) + np.array([rx, ry, rz])
        
        # --- D. Calculate Tag Orientation ---
        # The tag should face the robot.
        # Robot faces "Forward" (+X). Tag should face "Backward" (-X) relative to robot.
        # So Tag Orientation = Robot Orientation * 180_deg_Yaw
        
        # Rotate 180 degrees around Z axis
        rot_180_z = R.from_euler('z', 90, degrees=True)
        tag_quat = (rot_matrix * rot_180_z).as_quat() # [x, y, z, w]

        # --- E. Send Command to Gazebo ---
        req = SetEntityState.Request()
        req.state.name = 'apriltag_1' 
        req.state.reference_frame = 'world'
        
        req.state.pose.position = Point(x=world_point[0], y=world_point[1], z=world_point[2])
        req.state.pose.orientation = Quaternion(x=tag_quat[0], y=tag_quat[1], z=tag_quat[2], w=tag_quat[3])

        future = self.client.call_async(req)

def main():
    rclpy.init()
    node = RelativeTagMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()