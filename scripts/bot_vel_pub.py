#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32
from geometry_msgs.msg import Pose,Vector3
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from math import pi ,sin ,cos
import math
from squaternion import Quaternion as qn

class Odometry_node(Node):

    def __init__(self):
        super().__init__('odometry')

        self.cpr = 125 #encoder counts per revolution
        self.wheel_dia = .125 #meters
        self.wheel_separation = 0.43 #meters
        self.ticksPerMeter = self.cpr/(self.wheel_dia*3.14)

        timer_period = 0.05  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bot_vel_pub = self.create_publisher(TwistStamped, "bot_vel", 1)
        self.odom_pub = self.create_publisher(Odometry, "odom", 1)
        self.tf_pub = TransformBroadcaster(self)

        self.left_wheel_sub = self.create_subscription(Float32, "left_tickrate", self.left_wheel_callback, 1)
        self.left_wheel_vel = 0.0
        self.right_wheel_sub = self.create_subscription(Float32, "right_tickrate", self.right_wheel_callback, 1)
        self.right_wheel_vel = 0.0

        self.tick_sub = self.create_subscription(Vector3, "tick_count", self.tick_callback, 1)
        

        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.prev_time = self.get_clock().now().to_msg().sec
        self.new_tick_left = 0
        self.new_tick_right = 0

        self.prev_tick_left = 0
        self.prev_tick_right = 0

        self.pose = Pose()

    def odom_update(self):
        curr_time = self.get_clock().now().to_msg().sec
        dt = self.prev_time - curr_time
        vl = self.left_wheel_vel
        vr = self.right_wheel_vel
        v = (vr + vl) / 2.0
        w = (vr - vl) / self.wheel_separation

        bot_vel = TwistStamped()
        bot_vel.twist.linear.x = v
        bot_vel.twist.angular.z = w

        deltaX = v * math.cos(self.theta) * dt
        deltaY = v * math.sin(self.theta) * dt
        deltaTheta = w * dt
        self.x += deltaX
        self.y += deltaY    
        self.theta += deltaTheta

        q = qn.from_euler(0,0,self.theta)


        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q.x
        tf.transform.rotation.y = q.y
        tf.transform.rotation.z = q.z
        tf.transform.rotation.w = q.w

        self.tf_pub.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q.x
        odom.pose.pose.orientation.y = q.y
        odom.pose.pose.orientation.z = q.z
        odom.pose.pose.orientation.w = q.w
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)


        time = self.get_clock().now().to_msg()
        bot_vel.header.stamp = time
        bot_vel.header.frame_id = "odom"
        self.bot_vel_pub.publish(bot_vel)
        self.prev_time = curr_time

        pass

    def update_odom(self):

        # vl = self.left_wheel_vel
        # vr = self.right_wheel_vel
        # v = (vr + vl) / 2.0
        # w = (vr - vl) / self.wheel_separation

        curr_time = self.get_clock().now().to_msg().sec
        left_delta = self.new_tick_left - self.prev_tick_left
        right_delta = self.new_tick_right - self.prev_tick_right
        
        leftTravel = left_delta / self.ticksPerMeter
        rightTravel = right_delta / self.ticksPerMeter
        deltaTime = dt = self.prev_time - curr_time

        vl = leftTravel/dt
        vr = rightTravel/dt
        v = (vr + vl) / 2.0
        w = (vr - vl) / self.wheel_separation        

        deltaTravel = (rightTravel + leftTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / self.wheel_separation
        if rightTravel == leftTravel:

            deltaX = leftTravel*cos(self.theta)
            deltaY = leftTravel*sin(self.theta)
        else:
            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.x - radius*sin(self.theta)
            iccY = self.y + radius*cos(self.theta)

            deltaX = cos(deltaTheta)*(self.x - iccX) \
                - sin(deltaTheta)*(self.y - iccY) \
                + iccX - self.x

            deltaY = sin(deltaTheta)*(self.x - iccX) \
                + cos(deltaTheta)*(self.y - iccY) \
                + iccY - self.y

        self.x += deltaX
        self.y += deltaY
        self.theta = (self.theta + deltaTheta) % (2*pi)


        q = qn.from_euler(0,0,self.theta)


        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q.x
        tf.transform.rotation.y = q.y
        tf.transform.rotation.z = q.z
        tf.transform.rotation.w = q.w

        self.tf_pub.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q.x
        odom.pose.pose.orientation.y = q.y
        odom.pose.pose.orientation.z = q.z
        odom.pose.pose.orientation.w = q.w
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)


        time = self.get_clock().now().to_msg()
        bot_vel = TwistStamped()
        bot_vel.header.stamp = time
        bot_vel.header.frame_id = "odom"
        self.bot_vel_pub.publish(bot_vel)


        self.prev_tick_left = self.new_tick_left
        self.prev_tick_right = self.new_tick_right

        self.prev_time = curr_time

        pass

    def left_wheel_callback(self,msg:Float32):
        self.left_wheel_vel=msg.data/self.cpr*self.wheel_dia*3.14
        print(self.left_wheel_vel)

        pass
    def right_wheel_callback(self,msg:Float32):
        self.right_wheel_vel=msg.data/self.cpr*self.wheel_dia*3.14
        pass
    def tick_callback(self,msg:Vector3):
        self.new_tick_left = msg.x
        self.new_tick_right = msg.y
        pass

def main(args=None):
    rclpy.init(args=args)
    odometry_node = Odometry_node()

    while rclpy.ok():
        odometry_node.update_odom()
        rclpy.spin_once(odometry_node)

    # rclpy.spin(odometry_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()