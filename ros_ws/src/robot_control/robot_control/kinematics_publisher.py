#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from math import pi, sqrt, atan2

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
        vel_msg = Float64MultiArray()
        pos_msg = Float64MultiArray()
        vel_msg.data = self.wheel_vel
        pos_msg.data = self.swerve_pos 
        self.pub_vel.publish(vel_msg)
        self.pub_pos.publish(pos_msg)

    def cmdvel_callback(self, msg):

        Vx = 0.0
        Vy = 0.0
        W = 0.0
        theta = 0.0
        
        Vy = msg.linear.y
        Vx = msg.linear.x
        W = msg.angular.z
        
        V = sqrt(Vx**2 + Vy**2) 
        
        if Vy == 0:
            if Vx > 0:
                theta = pi/2
            elif Vx < 0:
                theta = -pi/2
        else:
            theta = atan2(Vx,Vy)
                   
        self.wheel_vel = [V, V, V, V]
        self.swerve_pos = [theta, theta, theta, theta]
        

def main(args=None):
    rclpy.init(args=args)

    kinematics_publisher = KinematicsPublisher()

    rclpy.spin(kinematics_publisher)

    kinematics_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
