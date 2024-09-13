#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32
from geometry_msgs.msg import Pose,Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from math import pi ,sin ,cos
import math
from squaternion import Quaternion as qn
import numpy as np

class LQR():
    def __init__(self):
        super().__init__('LQR')
        #Supress Scientific notation
        np.set_printoptions(precision=3, suppress=True)

        #maximum velocities
        self.maximum_linear_accelaration=1
        self.maximum_angular_accelaration=pi/4
        
    
    def lqr(actual_state,desired_state,Q,R,A,B,deltat):


        error=actual_state-desired_state

        N=10
        
        P=[None]*(N+1)
        Qf=Q
        P[N]=Qf
        for i in range(N, 0, -1):
        # Discrete-time Algebraic Riccati equation to calculate the optimal 
        # state cost matrix
            P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)

        # Create a list of N elements
        K = [None] * N
        u = [None] * N

        for i in range(N):
        # Calculate the optimal feedback gain K
            K[i] = -np.linalg.pinv(R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ A
            u[i] = K[i] @ error
 
        # Optimal control input is u_star
        u_star = u[N-1]
 
        return u_star

    def core(self,v,w,vf,wf):
        #Time Interval
        deltat=0.25 
        actual_state=np.array([[v],[w]])
        desired_state=np.array([[0],[0]])
        #A Matrix
        A=np.array([[1,0],[0,1]])
        #B Matrix
        B=np.array([[deltat, 0],[0, deltat]])

        R = np.array([[0.01,   0],  # Penalty for linear accelaration effort
                [  0, 0.01]])  # Penalty for angular accelaration effort
        
        Q=np.array([[1, 0],  # Penalty for linear velocity error
                    [0, 1]]) # Penalty for angular velocity error
        
        optimal_control_input=self.lqr(actual_state,desired_state,Q,R,A,B,deltat)

        return optimal_control_input



class diff_drive_ctrl(Node):
    def __init__(self):
        super.__init__('diff_drive_controller')
        self.cpr = 125 #encoder counts per revolution
        self.wheel_dia = .125 #meters
        self.wheel_separation = 0.43 #meters
        self.ticksPerMeter = self.cpr/(self.wheel_dia*3.14)

        self.v_setpoint = 0
        self.w_setpoint = 0
        self.v_measured = 0
        self.w_measured = 0
        
        self.left_pub = self.create_publisher(Float32, "left_wheel/control_effort",1)
        self.right_pub= self.create_publisher(Float32, "right_wheel/control_effort",1)

        # self.pid_sub_v = self.create_subscription(Vector3, "pid_tune_v", self.pid_callback_v, 1)
        # self.pid_sub_w = self.create_subscription(Vector3, "pid_tune_w", self.pid_callback_w, 1)

        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 1)

    def recv_cmd_vel(self,msg:Twist):
        self.bot_vel = msg
        
        self.v_setpoint = self.bot_vel.linear.x
        self.w_setpoint =self.bot_vel.angular.z
        pass

    def pid_callback_w(self,msg:Vector3):
        self.kp_w = msg.x
        self.ki_w = msg.y
        self.kd_w = msg.z
        pass

    def pid_callback_v(self,msg:Vector3):
        self.kp_v = msg.x
        self.ki_v = msg.y
        self.kd_v = msg.z
        pass

    def odom_callback(self,msg:Odometry):
        self.v_measured = msg.twist.twist.linear.x
        self.w_measured = msg.twist.twist.angular.z

def main(args=None):
    rclpy.init(args=args)

    ctrlr = diff_drive_ctrl()
   
    pid_v = pid_w = PID(0,0,0,setpoint=0)

    pid_v.output_limits(-2,2)
    pid_w.output_limits(-1,1)
    max_pwm_value = 512

    while rclpy.ok():

        rclpy.spin_once(ctrlr)

        w_m = ctrlr.w_measured
        v_m = ctrlr.v_measured

        w_sp = ctrlr.w_setpoint
        v_sp = ctrlr.v_setpoint

        pid_v.Kd = ctrlr.kd_v
        pid_v.Ki = ctrlr.ki_v
        pid_v.Kp = ctrlr.kp_v

        pid_w.Kd = ctrlr.kd_w
        pid_w.Ki = ctrlr.ki_w
        pid_w.Kp = ctrlr.kp_w

        linear_velocity = pid_v(v_m-v_sp)
        angular_velocity = pid_w(w_m-w_sp)

        pwm_r=(linear_velocity+angular_velocity)*max_pwm_value
        pwm_l=(linear_velocity-angular_velocity)*max_pwm_value
        
        ctrlr.left_pub.publish(pwm_l)
        ctrlr.right_pub.publish(pwm_r)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ctrlr.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
