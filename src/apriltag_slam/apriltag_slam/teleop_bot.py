#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import termios
import tty

# Renamed to avoid conflict with local variables
INSTRUCTIONS = """
Control Your Apriltag Torso Bot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease forward torque (gas/brake)
a/d : increase/decrease steering angle
s   : force stop (0 torque)
space: reset steering to 0

CTRL-C to quit
"""

settings = termios.tcgetattr(sys.stdin)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_bot')
        
        # Publishers
        self.drive_pub = self.create_publisher(
            Float64MultiArray, 
            '/drive_controller/commands', 
            10)
            
        self.steer_pub = self.create_publisher(
            JointTrajectory, 
            '/steering_controller/joint_trajectory', 
            10)

        # State
        self.torque = 0.0
        self.steering_angle = 0.0
        
        # Limits / Steps
        self.TORQUE_STEP = 0.5
        self.MAX_TORQUE = 5.0
        self.STEER_STEP = 0.1
        self.MAX_STEER = 0.8 # radians (~45 degrees)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_drive(self):
        msg = Float64MultiArray()
        # Apply same torque to both rear wheels
        msg.data = [self.torque, self.torque]
        self.drive_pub.publish(msg)

    def publish_steer(self):
        msg = JointTrajectory()
        msg.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.steering_angle, self.steering_angle]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 200ms
        
        msg.points = [point]
        self.steer_pub.publish(msg)

    def run(self):
        print(INSTRUCTIONS)
        try:
            while True:
                key = self.getKey()
                
                if key == 'w':
                    self.torque += self.TORQUE_STEP # Adjust sign if needed
                elif key == 'x':
                    self.torque -= self.TORQUE_STEP
                elif key == 'a':
                    self.steering_angle += self.STEER_STEP
                elif key == 'd':
                    self.steering_angle -= self.STEER_STEP
                elif key == 's':
                    self.torque = 0.0
                    self.steering_angle = 0.0
                elif key == ' ':
                    self.steering_angle = 0.0
                elif key == '\x03': # CTRL-C
                    break
                
                
                # Clamp values
                self.torque = max(min(self.torque, self.MAX_TORQUE), -self.MAX_TORQUE)
                self.steering_angle = max(min(self.steering_angle, self.MAX_STEER), -self.MAX_STEER)

                # Publish commands
                self.publish_drive()
                self.publish_steer()
                
                print(f"\rTorque: {self.torque:.2f} | Steer: {self.steering_angle:.2f}", end='')

        except Exception as e:
            print(e)

        finally:
            # Stop robot on exit
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0]
            self.drive_pub.publish(stop_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()