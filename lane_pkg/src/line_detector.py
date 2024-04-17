#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from warper import Warper
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Int64, Float64
from morai_msgs.msg import GPSMessage, CtrlCmd
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from math import *
from slidewindow import SlideWindow
from cv_bridge import CvBridge



class LineDetector() :

    def __init__(self) :
        rospy.init_node("line_detector")


        # rospy.Subscriber("/gps", GPSMessage, self.gpsCallback)
        # rospy.Subscriber("/imu", Imu, self.ImuCallback)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.imageCallback)
        # rospy.Subscriber("/waypoint", Int64, self.waypointCallback)

        # self.ctrl_cmd_pub = rospy.Publisher('/s_ctrl_cmd', CtrlCmd, queue_size=1)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)

        self.bridge = CvBridge()
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        # self.ctrl_cmd_msg = CtrlCmd()
        # self.odom_msg = Odometry()


        self.original_img = []
        self.yaw = 0.0
        self.waypoint = 0


        # 슬라이딩 윈도우 변수 초기화
        self.slide_img = None 
        self.slide_x_location = 0.0
        self.current_lane_window = ""

        self.initialized = False # 이미지 콜백

        self.gap = 0
        self.steering_angle = 0

        # 출력위해 만든 값
        self.epoch = 0

        


        # 지피에스 신호
        self.original_longitude = 0.0
        self.original_latitude = 0.0

        # 시험용
        # self.ctrl_cmd_msg.longlCmdType = 2
        self.motor_msg = 5.0
        self.servo_msg = 0.0 
        rate = rospy.Rate(20)

        
        while not rospy.is_shutdown():
            
            self.publishCtrlCmd(self.motor_msg, self.servo_msg)
            rate.sleep()
        
        rospy.spin()

    # def waypointCallback(self, msg) :
    #     self.waypoint = msg.data
        
        
    # def gpsCallback(self, msg): 
    #     self.original_longitude = msg.longitude
    #     self.original_latitude = msg.latitude
    
    # def ImuCallback(self, msg) :
    #     self.odom_msg.pose.pose.orientation.x = msg.orientation.x
    #     self.odom_msg.pose.pose.orientation.y = msg.orientation.y
    #     self.odom_msg.pose.pose.orientation.z = msg.orientation.z
    #     self.odom_msg.pose.pose.orientation.w = msg.orientation.w

    #     quaternion = (self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w)
    #     _, _, self.yaw =  euler_from_quaternion(quaternion)
    #     self.yaw = degrees(self.yaw)

        # self.epoch += 1
        # if self.epoch % 100 == 0:
        #     print(self.yaw)

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 마우스 왼쪽 버튼 클릭 이벤트 감지
            print(f'클릭한 좌표: ({x}, {y})')


    def imageCallback(self, _data) :
        if (self.original_longitude  <= 1 and self.original_latitude <=  1):
            cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)

            hsv_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

            lower_lane = np.array([0, 0, 126]) #0 46 170 / white lane: 0 0 126
            upper_lane = np.array([255, 64, 180]) #79 255 255 / white lane : 255, 64, 180


            lane_mask = cv2.inRange(hsv_image, lower_lane, upper_lane)

            ksize = 5
            blur_lane = cv2.GaussianBlur(lane_mask, (ksize, ksize), 0)

            _, lane_image = cv2.threshold(blur_lane, 200, 255, cv2.THRESH_BINARY)

            # warper_image = self.warper.warp(lane_image)
            cv2.imshow("out", lane_image)
            # self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warper_image, self.yaw)

            # cv2.imshow("slide_img", self.slide_img)
            # cv2.setMouseCallback('slide_img', self.click_event)

            if self.yaw <= -30 :
                self.slide_x_location += 80 
            cv2.waitKey(1)

            # self.gap = 320 - self.slide_x_location 
            # if self.slide_x_location > 400:
            #     self.motor_msg = 10
            #     self.steering_angle = self.gap * 0.004
            # elif self.slide_x_location < 270 or self.slide_x_location > 370:
            #     self.motor_msg = 15
            #     self.steering_angle = self.gap * 0.004
            # else:
            #     self.motor_msg = 20
            #     self.steering_angle = self.gap * 0.004
    
    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)

def nothing(x):
    pass


if __name__ == "__main__":
    line_detect = LineDetector()