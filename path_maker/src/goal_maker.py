#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys
import rospy
import rospkg
from math import pi, sqrt, atan2
from geometry_msgs.msg import PoseWithCovarianceStamped

warnings.simplefilter(action='ignore', category=FutureWarning)

class PathMaker :

    def __init__(self):
        rospy.init_node('goal_maker', anonymous=True)

        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
    
        self.cnt = 0
        
        self.prev_x = 0
        self.prev_y = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0

        self.is_amcl_pose = False

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')

        full_path = pkg_path + '/' + self.path_folder_name+'/' + self.make_path_name+'.txt'
        
        self.f = open(full_path, 'w')


        rate = rospy.Rate(50) 
        while not rospy.is_shutdown():
            if self.is_amcl_pose == True:
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):

        distance = sqrt(pow(self.x - self.prev_x, 2) + pow(self.y - self.prev_y, 2))

        if distance > 0.001:   #0.3
            data='{0}\t{1}\t{2}\t{3}\n'.format(self.x, self.y, self.z, self.w)

            self.f.write(data)

            self.prev_x = self.x
            self.prev_y = self.y
            self.cnt += 1
        
            print(self.cnt, self.x, self.y, self.z, self.w)
        

    def amcl_pose_callback(self, data): 
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.orientation.z
        self.w = data.pose.pose.orientation.w
        
        self.is_amcl_pose = True

        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass



