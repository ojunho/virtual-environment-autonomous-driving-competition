#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys
import rospy
import rospkg
from math import pi, sqrt, atan2
from geometry_msgs.msg import PoseWithCovarianceStamped
from morai_msgs.msg import EgoVehicleStatus

warnings.simplefilter(action='ignore', category=FutureWarning)

class PathMaker :

    def __init__(self):
        rospy.init_node('goal_maker', anonymous=True)

        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
    
        self.cnt = 0
        
        self.amcl_prev_x = 0
        self.amcl_prev_y = 0
        self.amcl_x = 0
        self.amcl_y = 0
  
        self.is_ego = False
        self.is_amcl_pose = False

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')

        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        
        self.f = open(full_path, 'w')


        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            if self.is_ego == True and self.is_amcl_pose == True:
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):

        distance = sqrt(pow(self.amcl_x - self.amcl_prev_x, 2) + pow(self.amcl_y - self.amcl_prev_y, 2))

        if self.amcl_x != self.amcl_prev_x:   #0.3
            data='{0}\t{1}\t{2}\t{3}\n'.format(self.amcl_x, self.ego_x, self.amcl_y, self.ego_y)

            self.f.write(data)

            self.amcl_prev_x = self.amcl_x
            self.amcl_prev_y = self.amcl_y
            self.cnt += 1
        
            print(self.amcl_x, self.ego_x)
        

    def amcl_pose_callback(self, data): 
        self.amcl_x = data.pose.pose.position.x - 19.0
        self.amcl_y = data.pose.pose.position.y + 5.0
        
        self.is_amcl_pose = True

    def ego_callback(self, data): 
        self.ego_x = data.position.x
        self.ego_y = data.position.y
        
        self.is_ego = True


        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass



