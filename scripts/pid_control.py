#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32
from geometry_msgs.msg import Pose,Twist,Vector3
from simple_pid import PID

class diff_drive_ctrl(Node):
    def __init__(self):
        super.__init__('diff_drive_controller')
        self.cpr = 125 #encoder counts per revolution
        self.wheel_dia = .125 #meters
        self.wheel_separation = 0.43 #meters
        self.ticksPerMeter = self.cpr/(self.wheel_dia*3.14)

        self.kp = 0
        self.ki = 0
        self.kd = 0 

        self.t_vl = 0
        self.t_vr = 0

        self.left_wheel_sub = self.create_subscription(Float32, "left_tickrate", self.left_wheel_callback, 1)
        self.right_wheel_sub = self.create_subscription(Float32, "right_tickrate", self.right_wheel_callback, 1)

        self.left_pub = self.create_publisher(Float32, "left_wheel/control_effort",1)
        self.right_pub = self.create_publisher(Float32, "right_wheel/control_effort",1)

        self.pid_sub = self.create_subscription(Vector3, "pid_tune", self.pid_callback, 1)

        self.c_vl = 0
        self.c_vr = 0

        self.bot_vel = Twist()
    
    def recv_cmd_vel(self,msg:Twist):
        self.bot_vel = msg
        
        v = self.bot_vel.linear.x
        w =self.bot_vel.angular.z
        
        self.t_vl = v - (w*self.wheel_separation/2)
        self.t_vr = v + (w*self.wheel_separation/2)
        pass

    def left_wheel_callback(self,msg:Float32):
        self.c_vl=msg.data/self.cpr*self.wheel_dia*3.14
        pass
    def right_wheel_callback(self,msg:Float32):
        self.c_vr=msg.data/self.cpr*self.wheel_dia*3.14
        pass
    def pid_callback(self,msg:Vector3):
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z
        pass


def main(args=None):
    rclpy.init(args=args)

    ctrlr = diff_drive_ctrl()
   
    pidl = pidr = PID()

    pwm_r=Float32()
    pwm_l=Float32()

    pidl.output_limits(-5,5)
    pidr.output_limits(-5,5)

    while rclpy.ok():

        rclpy.spin_once(ctrlr)

        c_vl = ctrlr.c_vl
        c_vr = ctrlr.c_vr

        pidl.Kd = ctrlr.kd
        pidl.Ki = ctrlr.ki
        pidl.Kp = ctrlr.kp

        pidl.Kd = ctrlr.kd
        pidl.Ki = ctrlr.ki
        pidl.Kp = ctrlr.kp

        pidl.setpoint = ctrlr.t_vl
        pidr.setpoint = ctrlr.t_vr

        ctrl_effort_left = pidl(c_vl)
        ctrl_effort_right = pidr(c_vr)
        pwm_l.data = ctrl_effort_left
        pwm_r.data = ctrl_effort_right
        
        ctrlr.left_pub.publish(pwm_l)
        ctrlr.right_pub.publish(pwm_r)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ctrlr.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
