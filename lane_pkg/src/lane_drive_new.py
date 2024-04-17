#!/usr/bin/python3
#-*- encoding: utf-8 -*-
import os.path as ops
# import json
import numpy as np
import torch
import cv2
import time
import math
import os
import matplotlib.pylab as plt
import sys
import rospy
import imageio
import tf
from tqdm import tqdm

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, Imu

from driving_es import *
# from slidewindow import *
from slidewindow2 import *
from std_msgs.msg import Int32, Float64
from std_msgs.msg import Bool
from tf.transformations import *




bridge = CvBridge()
image = np.empty(shape=[0])
last_angle = 0
def save_last_angle(angle):
     global last_angle
     last_angle = angle
# rospy.init_node("lane")

# torch.backends.cudnn.enabled = False

device = 'cuda' if torch.cuda.is_available() else 'cpu'
# device = 'cuda'

slidewindow = SlideWindow()
x_location = 320
last_x_location = 320
is_detected = True
current_lane = "LEFT"


def image_callback(img_data):
	global bridge
	global image
	image = bridge.compressed_imgmsg_to_cv2(img_data, desired_encoding="bgr8")

def drive2(angle, speed):
    speed = speed * 300
    motor_pub.publish(speed)
    servo_pub.publish(angle)

def callback(data):
    (r,p,y) = tf.transformations.euler_from_quaternion((data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w))
    # rospy.loginfo("Roll = %f, Pitch = %f, Yaw = %f",r*180/3.1415926,p*180/3.1415926,y*180/3.1415926)

def check_stopline(image):
    area = image[270:370, 280:480]
    cv2.imshow('crop_img', area)
    stopline_count = cv2.countNonZero(area)
    # print(stopline_count)

    ratio = stopline_count/(100*200)
    # print(ratio)
    if ratio > 0.28:
    # if ratio > 1:
        return True
    else:
        return False



if __name__ == "__main__":
    image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, image_callback)
    rospy.Subscriber("/imu", Imu, callback)
    

    lx,ly,rx,ry = [200,] , [] , [428,] , []  #sliding_Window 함수 내에서 x_temp , y_temp 매개변수에 lx[0]과 rx[0]을 할당해주어야 하기에 초기화 합니다.    

    rospy.init_node('lane_drive')

    print ("----- lane self driving -----")
    left_prob,right_prob = 0,0 #left_prob와 right_prob또한 slidingwindow 함수의 매개변수로 사용해야하기에 초기화합니다.

    x_location = 320
    last_x_location = 320
    is_detected = True
    current_lane = "LEFT"
    

        
    motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
    servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)

    while not rospy.is_shutdown():
        # try:
        show_img = image
        init_show_img(show_img)
        cv2.imshow("lane_image", show_img)
        cv2.waitKey(1)
        img_frame = show_img.copy() # img_frame변수에 카메라 이미지를 받아옵니다.   
        height,width,channel = img_frame.shape # 이미지의 높이,너비,채널값을 변수에 할당합니다. 

        # 240 한 이유: 하늘 볼 필요 없이 땅만 보려고 Y 기준 반 날림.
        img_roi = img_frame[240:,0:]   # y좌표 0~320 사이에는 차선과 관련없는 이미지들이 존재하기에 노이즈를 줄이기 위하여 roi설정을 해주었습니다.
        cv2.imshow("ROI", img_roi)

        
        img_filtered = color_filter(img_roi)   #roi가 설정된 이미지를 color_filtering 하여 흰색 픽셀만을 추출해냅니다. 
        cv2.imshow("filetered View", img_filtered)
        roied_height, roied_width, roied_channel = img_frame.shape
        img_warped = bird_eye_view(img_filtered, roied_width, roied_height) # 앞서 구현한 bird-eye-view 함수를 이용하여 시점변환해줍니다. 
        

        _, L, _ = cv2.split(cv2.cvtColor(img_warped, cv2.COLOR_BGR2HLS))
        _, img_binary = cv2.threshold(L, 0, 255, cv2.THRESH_BINARY) #color_filtering 된 이미지를 한번 더 이진화 하여 차선 검출의 신뢰도를 높였습니다. 

        img_masked = region_of_interest(img_binary) #이진화까지 마친 이미지에 roi를 다시 설정하여줍니다.
        out_img, x_location, current_lane= slidewindow.slidewindow(img_masked, is_detected)
        img_blended = cv2.addWeighted(out_img, 1, img_warped, 0.6, 0) # sliding window결과를 시각화하기 위하여 out_img와 시점변환된이미지를 merging 하였습니다.


        cv2.imshow("masked", img_masked)
        is_stopline = check_stopline(img_masked)
        # print(is_stopline)

        cv2.imshow("CAM View", img_blended)
        cv2.waitKey(1)

        if x_location == None :
            x_location = last_x_location
            is_detected = False
        else :
            last_x_location = x_location
            is_detected = True

        

        # print(is_detected)
        
        # if slidewindow.notleft_detect:
             
        #     angle = 0.08 #고정
        #     speed=4
        
        
        # else: 
        #     angle = (x_location-320)*0.005 + 0.5  #*3 값 angle 조정 0.005
        #     speed = 4
        
        

        #     if slidewindow.trigger and 0.65<=angle:
        #         angle = 0.4
            
        speed = 2
        angle = (x_location-320)*0.014  + 0.5
        # print(angle)

        if is_stopline == True:
            speed = 0
        
        
        drive2(angle, speed)

        # 직진: speed(4) angle_cal(0.005)
        # 로터리 코너: speed(2) angle_cal(0.015)

        
        
        
            




        # except:
        #     pass

        rospy.sleep(0.05)
