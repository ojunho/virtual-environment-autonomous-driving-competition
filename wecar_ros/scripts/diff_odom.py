#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import os
import sys
import rospy
import rospkg
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix, Imu
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage
from math import pi, sqrt, atan2
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
import tf


class DiffOdom:
    
    def __init__(self):

        rospy.init_node('diff', anonymous=True)


        rospy.Subscriber("/odometry/filtered", Odometry, self.odomCB)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.egoCB)


        self.ego_status = EgoVehicleStatus()
        self.odom_status = EgoVehicleStatus()

        self.ego_prev_x = 0
        self.ego_prev_y = 0

        self.odom_prev_x = 0
        self.odom_prev_y = 0

        self.cnt_odom = 0
        self.cnt_ego = 0

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.diff()
            rate.sleep()


    def odomCB(self, data): 
        self.odom_status.position.x = data.pose.pose.position.x
        self.odom_status.position.y = data.pose.pose.position.y
        if self.cnt_odom == 0:
            self.odom_prev_x = self.odom_status.position.x
            self.odom_prev_y = self.odom_status.position.y
            self.cnt_odom = 1
        # print(f"Odom: {self.odom_status.position.x}")

        

    def egoCB(self, msg):
        self.ego_status.position.x = msg.position.x
        self.ego_status.position.y = msg.position.y
        # print(f"Ego: {self.ego_status.position.x}")
        if self.cnt_ego == 0:
            self.ego_prev_x = self.ego_status.position.x
            self.ego_prev_y = self.ego_status.position.y
            self.cnt_ego = 1

        
    
    def diff(self):
        distance_ego = sqrt(pow(self.ego_status.position.x - self.ego_prev_x, 2) + pow(self.ego_status.position.y - self.ego_prev_y, 2))
        distance_odom = sqrt(pow(self.odom_status.position.x - self.odom_prev_x, 2) + pow(self.odom_status.position.y - self.odom_prev_y, 2))

        # self.odom_prev_x = self.odom_status.position.x
        # self.odom_prev_y = self.odom_status.position.y
        # self.ego_prev_x = self.ego_status.position.x
        # self.ego_prev_y = self.ego_status.position.y

        print(f"Ego: {distance_ego}\nOdom: {distance_odom}")

        

if __name__ == '__main__':
    try:
        diff_odom = DiffOdom()
    except rospy.ROSInterruptException:
        pass