#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import cv2
from cv_bridge import CvBridge
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R

class VisualOdometer:
    """
    Performs Monocular Visual Odometry using Epipolar Geometry.
    Tracks features frame-to-frame to estimate Rotation (R) and Direction (t).
    """
    def __init__(self):
        self.old_frame = None
        self.p0 = None
        # VO Parameters
        self.feature_params = dict(maxCorners=200, qualityLevel=0.01, minDistance=30, blockSize=7)
        self.lk_params = dict(winSize=(21, 21), maxLevel=3, 
                            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        
    def process(self, frame, K):
        """
        Returns:
            R_vo (3x3): Relative rotation from prev to curr
            t_vo (3x1): Relative unit translation vector
            valid (bool): Success flag
        """
        if self.old_frame is None:
            self.old_frame = frame
            # Detect features in first frame
            self.p0 = cv2.goodFeaturesToTrack(self.old_frame, mask=None, **self.feature_params)
            return np.eye(3), np.zeros(3), False

        # 1. Optical Flow (Track features)
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_frame, frame, self.p0, None, **self.lk_params)

        # Select good points
        if p1 is not None and len(p1) > 10:
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]
        else:
            # Lost track, reset
            self.old_frame = frame
            self.p0 = cv2.goodFeaturesToTrack(self.old_frame, mask=None, **self.feature_params)
            return np.eye(3), np.zeros(3), False

        # 2. Epipolar Geometry (Find Essential Matrix)
        # focal = K[0,0], pp = (K[0,2], K[1,2])
        E, mask = cv2.findEssentialMat(good_new, good_old, focal=K[0,0], pp=(K[0,2], K[1,2]), 
                                     method=cv2.RANSAC, prob=0.999, threshold=1.0)
        
        R_vo = np.eye(3)
        t_vo = np.zeros(3)
        valid = False

        if E is not None and E.shape == (3,3):
            # 3. Recover Pose (R, t) from E
            _, R_rec, t_rec, _ = cv2.recoverPose(E, good_new, good_old, focal=K[0,0], pp=(K[0,2], K[1,2]))
            R_vo = R_rec
            t_vo = t_rec.flatten() # Note: This has UNIT scale (length = 1)
            valid = True

        # Update for next step
        self.old_frame = frame.copy()
        
        # Replenish features if dropping too low
        if len(good_new) < 70:
            self.p0 = cv2.goodFeaturesToTrack(self.old_frame, mask=None, **self.feature_params)
        else:
            self.p0 = good_new.reshape(-1, 1, 2)
            
        return R_vo, t_vo, valid

class SLAMNode(Node):
    def __init__(self):
        super().__init__('torso_slam_node')
        
        # --- State ---
        self.p = np.zeros(3)  # Camera Position (World)
        self.v = np.zeros(3)  # Camera Velocity (World)
        self.q = np.array([0.0, 0.0, 0.0, 1.0]) # Camera Orientation
        
        # --- Map ---
        # 0: Static Reference Tag (Fixed at origin)
        # 1: Moving Target Tag (We want to track this)
        self.static_tag_id = 0
        self.static_tag_pos = np.array([1.0, 0.0, 0.5]) # Known location of Tag 0
        
        # --- Modules ---
        self.vo = VisualOdometer()
        self.bridge = CvBridge()
        self.detector = None
        self.camera_matrix = None # K matrix (3x3)
        self.K_list = None        # [fx, fy, cx, cy]
        
        # --- Calibration ---
        self.ba = np.zeros(3)
        self.calibrated = False
        self.calib_samples = []
        self.last_imu_time = None

        # --- Subscribers ---
        self.create_subscription(Imu, '/imu_broadcaster/imu', self.imu_callback, 10)
        self.create_subscription(CameraInfo, '/apriltag_slam/camera/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/apriltag_slam/camera/image_raw', self.image_callback, 10)
        
        # --- Publishers ---
        # Camera Pose (Our SLAM result)
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag_slam/camera/pose', 10)
        # Moving Tag Pose (The object we are tracking)
        self.target_pub = self.create_publisher(PoseStamped, '/apriltag_slam/target_tag/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("Robust SLAM Node Started.")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            k = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = k
            self.K_list = [k[0,0], k[1,1], k[0,2], k[1,2]]
            self.detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0, quad_sigma=0.8)

    def imu_callback(self, msg):
        if self.last_imu_time is None:
            self.last_imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            return
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = curr_time - self.last_imu_time
        self.last_imu_time = curr_time
        
        # 1. IMU Pre-integration
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        if not self.calibrated:
            self.calib_samples.append(acc)
            if len(self.calib_samples) >= 100:
                self.ba = np.mean(self.calib_samples, axis=0) - np.array([0,0,9.81])
                self.calibrated = True
                self.get_logger().info("IMU Calibrated.")
            return

        # 2. Rotation Update (High Frequency)
        rot = R.from_quat(self.q)
        dq = R.from_rotvec(gyro * dt)
        self.q = (rot * dq).as_quat()

        # 3. Position Prediction (IMU Dead Reckoning)
        acc_world = rot.as_matrix() @ (acc - self.ba) - np.array([0,0,9.81])
        if np.linalg.norm(acc_world) < 0.05: acc_world = np.zeros(3)
        
        self.v += acc_world * dt
        self.p += self.v * dt
        
        # Publish "Fast" Estimate
        self.publish_pose(msg.header.stamp)

    def image_callback(self, msg):
        if not self.calibrated or self.camera_matrix is None: return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except: return

        # --- A. VISUAL ODOMETRY (Relative Motion) ---
        R_vo, t_vo_unit, valid_vo = self.vo.process(cv_img, self.camera_matrix)
        
        if valid_vo:
            # Scale Ambiguity: t_vo is unit length.
            # Strategy: Use IMU velocity magnitude as a hint for scale
            current_speed = np.linalg.norm(self.v)
            dt_cam = 0.033 # Approx 30Hz
            scale = current_speed * dt_cam 
            
            # Apply VO rotation to smooth IMU drift
            # (Simplified fusion: trusting IMU orientation more, but VO helps)
            
            # Apply VO Translation (Scaled)
            # Transform t_vo (Camera Frame) -> World Frame
            rot_mat = R.from_quat(self.q).as_matrix()
            t_world = rot_mat @ t_vo_unit 
            
            # Weighted Fusion: 10% VO, 90% IMU inertia
            # This helps smoothen the trajectory
            if scale > 0.001:
                self.p += t_world * scale * 0.1

        # --- B. APRILTAGS (Absolute Correction) ---
        tags = self.detector.detect(cv_img, estimate_tag_pose=True, 
                                  camera_params=self.K_list, tag_size=0.15)
        
        static_tag_seen = False
        
        for tag in tags:
            # 1. Transform Tag Pose: Camera -> Body (Z-fwd to X-fwd)
            t_opt = tag.pose_t.flatten()
            t_body = np.array([t_opt[2], -t_opt[0], -t_opt[1]])
            rot_mat = R.from_quat(self.q).as_matrix()
            t_world_rel = rot_mat @ t_body

            if tag.tag_id == self.static_tag_id:
                # --- STATIC TAG: Correct Camera Pose ---
                static_tag_seen = True
                
                # Camera = Tag_Known - Relative_Vector
                pos_correction = self.static_tag_pos - t_world_rel
                
                # Strong correction (SLAM Loop Closure)
                alpha = 0.4
                self.p = (1 - alpha) * self.p + alpha * pos_correction
                self.v *= 0.95 # Dampen velocity drift
                
            else:
                # --- MOVING TAG: Track its Position ---
                # Tag_World = Camera_World + Relative_Vector
                tag_world_pos = self.p + t_world_rel
                self.publish_target(tag_world_pos, tag.tag_id, msg.header.stamp)

    def publish_pose(self, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.pose.position.x = self.p[0]
        msg.pose.position.y = self.p[1]
        msg.pose.position.z = self.p[2]
        msg.pose.orientation.x = self.q[0]
        msg.pose.orientation.y = self.q[1]
        msg.pose.orientation.z = self.q[2]
        msg.pose.orientation.w = self.q[3]
        self.pose_pub.publish(msg)
        
        # TF
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = "camera_link_estimated"
        t.transform.translation.x = self.p[0]
        t.transform.translation.y = self.p[1]
        t.transform.translation.z = self.p[2]
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_target(self, pos, tag_id, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        # Identity orientation for the point
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

def main():
    rclpy.init()
    node = SLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()