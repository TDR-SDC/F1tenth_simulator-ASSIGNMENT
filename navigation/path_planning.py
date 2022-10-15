#! /usr/bin/env python3
import math
import numpy as np
import rospy
from f1tenth_simulator.msg import PIDInput
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt

angle_range_lidar = 270
# angle_range = np.pi
car_length = 1.5

desired_trajectory = 0

# write publisher for custom message

## ***********************CREATE A CUSTOM .MSG named PIDinput.msg in msg directory ****** 
# should have atleast ,pid_vel,pid_error, Feel free to add any more if required


def getRange(data, angle):
    # Get lidar range data in form of a list from the message


def Path_Planning():
    pass
    # decide the deviation from correct path publish in PID as message
    # write algorithm that give error to Controls script for steering


def callback(data):
    # Function to call other modules

    # publish custom message here

if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()
