#!/usr/bin/env python3

import copy
import time
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy



MIN_PWM1 = 0.0
BASE_PWM1 = 40.0
MAX_PWM1 = 255.0

MIN_PWM2 = 0.0
BASE_PWM2 = 80.0
MAX_PWM2 = 255.0


# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch

class PIDController(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.axes=[]
        self.buttons=[]
        self.l=0.36  #width of the robot in meters



        self.pwm_r=Float32()
        self.pwm_l=Float32()
       
        self.joystick=self.create_subscription(Joy,"/joy",self.joystick_callback,1)
        self.left = self.create_publisher(Float32, "left_wheel/control_effort",1)
        self.right= self.create_publisher(Float32, "right_wheel/control_effort",1)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.joystick_control()

    
    def joystick_callback(self,msg):
        #print(msg.axes[0])
        self.axes=msg.axes
        self.buttons=msg.buttons


    def joystick_control(self):
        if(self.axes!=[]):
            y=-self.axes[1]
            x=-self.axes[0]
            max_joystick_value=1
            max_pwm_value=50
            linear_velocity=y*max_joystick_value
            angular_velocity=x*max_joystick_value
            
            vr=(linear_velocity+angular_velocity)*max_pwm_value
            vl=(linear_velocity-angular_velocity)*max_pwm_value
            self.pwm_r.data=vr
            self.pwm_l.data=vl
        self.left.publish(self.pwm_l)
        self.right.publish(self.pwm_r)
        
        
           

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PIDController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()