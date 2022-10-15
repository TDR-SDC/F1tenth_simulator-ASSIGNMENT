#! /usr/bin/env python3
import math

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_simulator.msg import PIDInput

pub = rospy.Publisher('nav', AckermannDriveStamped, queue_size=1)

kp = 
kd = 
kp_vel = 
kd_vel = 

ki =
servo_offset =
prev_error =
error = 
integral = 
vel_input = 


def control(data):
    global integral
    global prev_error
    global vel_input
    global kp
    global ki
    global kd
    global kd_vel
    global kp_vel

    #take pid Errors using subscriber and write control script to output velocity and steering angle
    
    print("Velocity", velocity)
    print("Angle", angle)
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'cmd_vel'
    msg.drive.speed = velocity
    msg.drive.steering_angle = angle
    pub.publish(msg)


def listener():
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", PIDInput, control)
    
    rospy.spin()


if __name__ == '__main__':
    print("Listening to error for PID")
    listener()
