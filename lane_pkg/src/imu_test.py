#!/usr/bin/env python3
#coding=UTF-8

import rospy
import tf
from tf.transformations import *
from sensor_msgs.msg import Imu

def callback(data):
    (r,p,y) = tf.transformations.euler_from_quaternion((data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
    rospy.loginfo("Roll = %f, Pitch = %f, Yaw = %f",r*180/3.1415926,p*180/3.1415926,y*180/3.1415926)

def get_imu():
    rospy.init_node('get_imu', anonymous=True)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    get_imu()
