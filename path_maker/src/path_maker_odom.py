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

warnings.simplefilter(action='ignore', category=FutureWarning)

class PathMaker :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_filtered_callback)
        
        self.cnt = 0
        
        self.prev_x = 0
        self.prev_y = 0


        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f = open(full_path, 'w')

        rate = rospy.Rate(50) 
        while not rospy.is_shutdown():
            self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        x = self.odom_x
        y = self.odom_y
        
        # print(x, y)

        distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))

        if distance > 0.01:   #0.3
            data='{0}\t{1}\n'.format(x, y)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y
            self.cnt += 1
        
            print(self.cnt, 'X:', x, 'Y:', y)
            
        

    def odom_filtered_callback(self, data): 
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
    
        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass



