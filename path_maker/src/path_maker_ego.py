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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import tf

warnings.simplefilter(action='ignore', category=FutureWarning)

class PathMaker :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback)

        self.cnt = 0
        
        self.is_ego = False
        self.prev_x = 0
        self.prev_y = 0

        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f = open(full_path, 'w')

        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            if self.is_ego == True:
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        distance = sqrt(pow(self.ego_x - self.prev_x, 2) + pow(self.ego_y - self.prev_y, 2))

        if distance > 0.05:   #0.3
            data='{0}\t{1}\t{2}\n'.format(self.ego_x, self.ego_y, 0)
            self.f.write(data)
            self.prev_x = self.ego_x
            self.prev_y = self.ego_y
            self.cnt += 1
        
            print(self.cnt, self.ego_x, self.ego_y)
            
            
    def ego_callback(self, data): 
        self.ego_x = data.position.x
        self.ego_y = data.position.y
        
        self.is_ego = True

        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass



