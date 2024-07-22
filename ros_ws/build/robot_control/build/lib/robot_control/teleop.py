#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class KinematicsPublisher(Node):
    
    def __init__(self):
        super().__init__("kinematics_publisher")
        # subscriber
        self.sub_vel = self.create_subscription(Twist, "cmd_vel", self.cmdvel_callback, 10)
        self.sub_vel
        # publisher
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        # timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # varieble
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0]
        self.swerve_pos = [0.0, 0.0, 0.0, 0.0]
        
    def timer_callback(self):    
        self.pub_vel.publish(Float64MultiArray(data=self.wheel_vel))
        self.pub_pos.publish(Float64MultiArray(data=self.swerve_pos))

    def cmdvel_callback(self, msg):
        global Vx,Vy,W
        
        Vx = msg.linear.x
        Vy = msg.linear.y
        W = msg.angular.z
        
        # wheel_vel is velocity of each wheel
        # wheel_vel = [fl, fr, rl, rr]
        self.wheel_vel = [1.0, 1.0, 1.0, 1.0]
        # swerve_pos is the position of each swerve position
        # swerve_pos = [fl, fr, rl, rr]
        self.swerve_pos = [45.0, 45.0, 0.0, 0.0]
        
        print("Vx : " + str(Vx))
        print("Vy : " + str(Vy))
        print("W : " + str(W))
        print("---------------------------")

def main(args=None):
    rclpy.init(args=args)

    kinematics_publisher = KinematicsPublisher()

    rclpy.spin(kinematics_publisher)

    kinematics_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
