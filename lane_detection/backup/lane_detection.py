#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy 
from morai_msgs.msg import GetTrafficLightStatus, EgoVehicleStatus
from std_msgs.msg import Float64, Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Imu
from obstacle_detection.msg import NearestObstacleInfo
from move_base_msgs.msg import MoveBaseActionResult
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2

from slidewindow_faster import SlideWindow

import time
import cv2
import numpy as np


class PID():
  def __init__(self,kp,ki,kd):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.p_error = 0.0
    self.i_error = 0.0
    self.d_error = 0.0

  def pid_control(self, cte):
    self.d_error = cte-self.p_error
    self.p_error = cte
    self.i_error += cte

    return self.kp*self.p_error + self.ki*self.i_error + self.kd*self.d_error

class  LaneDetection:
    def __init__(self):
        rospy.init_node("lane_detection_node")
        self.motor_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        
        self.stopline_pub = rospy.Publisher("/stopline_cnt", Int32, queue_size=1)
        
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        rospy.Subscriber("/imu", Imu, self.imu_CB)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB)

        rospy.Subscriber("/nearest_obstacle_info_dynamic_static", NearestObstacleInfo, self.obstacle_dynamic_static_CB)
        rospy.Subscriber("/nearest_obstacle_info_rotary", NearestObstacleInfo, self.obstacle_rotary_CB)

        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.goal_reached_CB)
        rospy.Subscriber("/nav_commands_motor_speed", Float64, self.nav_motor_CB)
        rospy.Subscriber("/nav_commands_servo_position", Float64, self.nav_steer_CB)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odomCB)


        #----------------------------- added for stop line count ------------------------------ #

        self.odom_status = EgoVehicleStatus()
        self.prev_odom = EgoVehicleStatus()

        self.stopline_cnt = 0

        self.cnt_odom = 0
        
        #-------------------------------------------------------------------------------------- #
        
        self.bridge = CvBridge()
        self.motor_msg = Float64()
        self.steer_msg = Float64()
        self.traffic_msg = GetTrafficLightStatus()

        self.slidewindow = SlideWindow()
        self.traffic_flag = 0
        self.prev_signal = 0
        self.signal = 0
        self.stopline_flag = 0
        self.img = []
        self.warped_img = []
        self.grayed_img = []
        self.out_img = []
        self.yellow_img = []
        self.white_img = []
        self.img_hsv = []
        self.h = []
        self.s = []
        self.v = []
        self.bin_img = []
        self.left_indices = []
        self.right_indices = []

        self.x_location = 320
        self.last_x_location = 320

        self.prev_center_index = 320
        self.center_index = 320
        self.standard_line = 320
        self.degree_per_pixel = 0

        self.is_slidewindow = False
        self.sliding_window_select_line = 'Right'

        self.status_msg_x = 0
        self.status_msg_y = 0
        self.status_msg_heading = 0

        self.yaw = 0

        self.is_curved_finished_at_five = False
        self.curved_index_at_five = 320

        self.is_curved_finished_at_six = False
        self.curved_index_at_six = 336

        self.is_curved_finished_at_seven = False
        self.curved_index_at_seven = 320

        
        #------------------ 정적 장애물 파라미터 ------------------#
        self.current_lane = "RIGHT"
        self.is_static_ob_passed = True
        self.curved_index_at_static = 320
        self.comeback_slidewindow = False
        self.comeback_right = False
        self.is_on_laneswitching = False
        
        self.reverse_betting_flag = True
        self.reverse_betting_pose = 112
        #---------------------------------------------------#
        
        #------------------ 로터리 파라미터 ------------------#
        self.rotary_brake_flag = False
        self.is_rotary_entered = False
        #---------------------------------------------------#
        
        #------------------ 신호등 파라미터 ------------------#
        self.is_green_left_light = False
        self.is_traffic_stopped = False
        self.is_traffic_passed = True
        #---------------------------------------------------#
        
        #------------------ SLAM 파라미터 ------------------#
        self.is_goal_arrived = False # False
        self.nav_motor_msg = 0
        self.nav_steer_msg = 0
        #---------------------------------------------------#


        #------------------ 라이다 파라미터 ------------------#
        self.obstacle_dynamic_static_x = 0
        self.obstacle_dynamic_static_y = 0

        self.obstacle_rotary_x = 0
        self.obstacle_rotary_y = 0

        self.finish_detection = False
        self.finish_second_detection = False
        self.is_available_to_go = False
        self.obstacle_y_list = []
        self.dynamic_obstacle_y_list = []

        self.is_dynamic = False
        self.is_static = False
        #----------------------------------------------------#

        #------------------ 이미지 파라미터 ------------------#
        self.left_hist_end_line = 640//8*3
        self.right_hist_start_line = 640//8*5

        self.up_hist_end_line = 480//4*3
        self.down_hist_start_line = 480//4*3
        #----------------------------------------------------#

        rate = rospy.Rate(30)  # hz 
        while not rospy.is_shutdown():
            # print(self.current_lane)
            # print(self.is_dynamic)
            # print(len(self.obstacle_y_list))
            # print(self.obstacle_dynamic_static_y)
            

            # print(f"TRAFFIC PASSED: {self.is_traffic_passed}")
            
            # print(f"STATIC:  {self.is_static}")
            # print(f"DYNAMIC: {self.is_dynamic}")
          
            
            # print("GOAL ARRIVED:", self.is_goal_arrived)
            # print('Finish Detection:', self.finish_detection)

            # print(f"status_msg_x: {self.status_msg_x}")
            # print(f"status_msg_y: {self.status_msg_y}")
            # print(f"status_msg_heading: {self.status_msg_heading}")
            # print(f"MODE: {self.stopline_cnt}")

            # print(f"self.is_static_ob_passed: {self.is_static_ob_passed}")
            # print(f"POSE: {self.reverse_betting_pose}")

            if self.is_goal_arrived == True and len(self.img)!= 0:
            # if len(self.img) != 0:
                self.stopline_pub.publish(self.stopline_cnt)

                y, x = self.img.shape[0:2]
                self.img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
                h, s, v = cv2.split(self.img_hsv)
                yellow_lower = np.array([15, 110, 80])
                yellow_upper = np.array([40, 255, 255])
                self.yellow_range = cv2.inRange(self.img_hsv, yellow_lower, yellow_upper)
                
                white_lower = np.array([0, 0, 100])
                white_upper = np.array([179, 64, 255])
                self.white_range = cv2.inRange(self.img_hsv, white_lower, white_upper)
                
                combined_range = cv2.bitwise_or(self.yellow_range, self.white_range)
                filtered_img = cv2.bitwise_and(self.img, self.img, mask=combined_range)
                
                src_point1 = [0, 420]      # 왼쪽 아래
                src_point2 = [235, 285]
                src_point3 = [x-235, 285]
                src_point4 = [x, 420]      # 오른쪽 아래
                src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
                
                dst_point1 = [x//8, 480]    # 왼쪽 아래
                dst_point2 = [x//8, 0]      # 왼쪽 위
                dst_point3 = [x//8*7, 0]    # 으론쪽 위
                dst_point4 = [x//8*7, 480]  # 오른쪽 아래
                dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])
                
                matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                self.warped_img = cv2.warpPerspective(filtered_img, matrix, [x,y])
                self.grayed_img = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2GRAY)
                
                
                
                # 이미지 이진화
                self.bin_img = np.zeros_like(self.grayed_img)
                self.bin_img[self.grayed_img > 150] = 1
                
                # 가로 선 먼저 탐지 후 차선 인식에 방해가 되지 않도록 가로로 평행한 정지선 픽셀들을 삭제
                histogram_y = np.sum(self.bin_img, axis=1)
                try:
                    histogram_y_indices = np.where(histogram_y > 300)[0]
                    stopline_start = histogram_y_indices[0] - 5
                    stopline_end = histogram_y_indices[-1] + 5
                    self.bin_img[stopline_start:stopline_end, :] = 0
                except:
                    pass
                
                # 세로 선 탐지
                histogram_x = np.sum(self.bin_img, axis=0)
                
                left_hist = histogram_x[0:self.left_hist_end_line]
                right_hist = histogram_x[self.right_hist_start_line:]
                
                self.left_indices = np.where(left_hist > 40)[0]
                self.right_indices = np.where(right_hist > 40)[0] + self.right_hist_start_line
                
                up_hist = histogram_y[0:self.up_hist_end_line]
                down_hist = histogram_y[self.down_hist_start_line:]
                
                stopline_indices = np.where(down_hist > 300)[0] + self.down_hist_start_line
                
                try:
                    stopline_threshold = 5
                    stopline_diff = stopline_indices[-1] - stopline_indices[0]
                    if stopline_threshold < stopline_diff:
                        self.stopline_flag = True
                        cv2.rectangle(self.warped_img, [0, stopline_indices[0]], [x, stopline_indices[-1]], [0,255,0], 3)
                    else:
                        self.stopline_flag = False
                except:
                    self.stopline_flag = False
                    
                indices = np.where(histogram_x > 40)[0]


                # 3번 정지선 못봐도 x, y, heading으로 갱신하도록.. !!
                if (12 <= self.status_msg_x) and (self.status_msg_y <= -2.5) and (self.status_msg_heading >= 30):
                    self.stopline_cnt = 3
                
                
                #----------------------------- added for stop line count ------------------------------ #
                if self.stopline_flag == True:
                    self.check_odom()

                #-------------------------------------------------------------------------------------- #
                


                if self.x_location == None :
                    self.x_location = self.last_x_location
                else :
                    self.last_x_location = self.x_location

                self.speed = 8
            
                ########################################## 0 0 0 0 0 ##########################################
                if self.stopline_cnt == 0:  # Finished
                    self.sliding_window_select_line = 'Right'
                    
                    #--------------------------------------------- 장애물 ---------------------------------------------#
                    if (0.4 < self.obstacle_dynamic_static_x < 1.5) and self.finish_detection == False:
                        self.publishMotorSteerMsg(0, self.steer_msg)
                        self.obstacle_y_list.append(self.obstacle_dynamic_static_y)

                        # if len(self.obstacle_y_list) >= 10 and self.finish_detection == False:
                        #     if abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]) > 0.08:
                        #         # self.obstacle_y_list.sort()
                        #         self.finish_detection = True
                        #         self.is_dynamic = True
                        #         self.is_static = False
                        
                        if len(self.obstacle_y_list) == 400 and self.finish_detection == False :
                            self.finish_detection = True
                            # self.obstacle_y_list.sort()
                
                            if abs(self.obstacle_y_list[100] - self.obstacle_y_list[-3]) > 0.08:
                                self.is_dynamic= True
                                self.is_static = False
                            # elif (-0.15 <= self.obstacle_y_list[0] <= 0.15 and -0.15 <= self.obstacle_y_list[-1] <= 0.15):
                            else:
                                self.is_static = True
                                self.is_dynamic = False

                        continue

                    if self.is_dynamic:
                        if -0.4 < self.obstacle_dynamic_static_y < 0.5 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                            self.publishMotorSteerMsg(0, self.steer_msg)

                            # if self.finish_second_detection == False:
                            #     self.dynamic_obstacle_y_list.append(self.obstacle_dynamic_static_y)

                            #     if len(self.dynamic_obstacle_y_list) >= 400 and self.finish_second_detection == False:
                            #         if abs(self.dynamic_obstacle_y_list[100] - self.dynamic_obstacle_y_list[-3]) <= 0.08:
                            #             self.finish_second_detection = True
                            #             self.is_static = True
                            #             self.is_dynamic = False
                                        


                            self.obstacle_y_list =[]
                            continue

                        elif self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0:
                            self.is_dynamic = False
                            self.obstacle_y_list = []
                            self.finish_detection = False

                        

                    elif self.is_static:
                        if self.finish_detection == True:
                            self.is_goal_arrived = False
                            self.is_static = False
                            self.obstacle_y_list = []
                            continue
                    

                    # 장애물을 지나갔다는 판단
                    if self.obstacle_dynamic_static_x < 0.1 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                        self.finish_detection = False
                        self.finish_second_detection = False
                        self.obstacle_y_list = []
                        self.dynamic_obstacle_y_list = []
                        self.is_dynamic = False
                        self.is_static = False
                    #--------------------------------------------------------------------------------------------------#

                    # 보편적인 상황
                    self.is_slidewindow = True

                # ########################################## 1 1 1 1 1 ##########################################
                elif self.stopline_cnt == 1:  # Finished
                    self.sliding_window_select_line = 'Right'

                    #--------------------------------------------- 장애물 ---------------------------------------------#
                    if (0.4 < self.obstacle_dynamic_static_x < 1.5) and self.finish_detection == False:
                        self.publishMotorSteerMsg(0, self.steer_msg)
                        self.obstacle_y_list.append(self.obstacle_dynamic_static_y)

                        # if len(self.obstacle_y_list) >= 10 and self.finish_detection == False:
                        #     if abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]) > 0.08:
                        #         # self.obstacle_y_list.sort()
                        #         self.finish_detection = True
                        #         self.is_dynamic = True
                        #         self.is_static = False
                        
                        if len(self.obstacle_y_list) == 400 and self.finish_detection == False :
                            self.finish_detection = True
                            # self.obstacle_y_list.sort()
                
                            if abs(self.obstacle_y_list[100] - self.obstacle_y_list[-3]) > 0.08:
                                self.is_dynamic= True
                                self.is_static = False
                            # elif (-0.15 <= self.obstacle_y_list[0] <= 0.15 and -0.15 <= self.obstacle_y_list[-1] <= 0.15):
                            else:
                                self.is_static = True
                                self.is_dynamic = False

                        continue

                    if self.is_dynamic:
                        if -0.4 < self.obstacle_dynamic_static_y < 0.5 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                            self.publishMotorSteerMsg(0, self.steer_msg)

                            # if self.finish_second_detection == False:
                            #     self.dynamic_obstacle_y_list.append(self.obstacle_dynamic_static_y)

                            #     if len(self.dynamic_obstacle_y_list) >= 400 and self.finish_second_detection == False:
                            #         if abs(self.dynamic_obstacle_y_list[100] - self.dynamic_obstacle_y_list[-3]) <= 0.08:
                            #             self.finish_second_detection = True
                            #             self.is_static = True
                            #             self.is_dynamic = False
                                        


                            self.obstacle_y_list =[]
                            continue

                        elif self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0:
                            self.is_dynamic = False
                            self.obstacle_y_list = []
                            self.finish_detection = False

                        

                    elif self.is_static:
                        if self.finish_detection == True:
                            self.is_goal_arrived = False
                            self.is_static = False
                            self.obstacle_y_list = []
                            continue
                    

                    # 장애물을 지나갔다는 판단
                    if self.obstacle_dynamic_static_x < 0.1 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                        self.finish_detection = False
                        self.finish_second_detection = False
                        self.obstacle_y_list = []
                        self.dynamic_obstacle_y_list = []
                        self.is_dynamic = False
                        self.is_static = False
                    #--------------------------------------------------------------------------------------------------#
                    
                    self.is_slidewindow = True


                # ########################################## 2 2 2 2 2 ##########################################
                elif self.stopline_cnt == 2:  # Finished
                    self.sliding_window_select_line = 'Right'

                    #--------------------------------------------- 장애물 ---------------------------------------------#
                    if (0.4 < self.obstacle_dynamic_static_x < 1.5) and self.finish_detection == False:
                        self.publishMotorSteerMsg(0, self.steer_msg)
                        self.obstacle_y_list.append(self.obstacle_dynamic_static_y)

                        # if len(self.obstacle_y_list) >= 10 and self.finish_detection == False:
                        #     if abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]) > 0.08:
                        #         # self.obstacle_y_list.sort()
                        #         self.finish_detection = True
                        #         self.is_dynamic = True
                        #         self.is_static = False
                        
                        if len(self.obstacle_y_list) == 400 and self.finish_detection == False :
                            self.finish_detection = True
                            # self.obstacle_y_list.sort()
                
                            if abs(self.obstacle_y_list[100] - self.obstacle_y_list[-3]) > 0.08:
                                self.is_dynamic= True
                                self.is_static = False
                            # elif (-0.15 <= self.obstacle_y_list[0] <= 0.15 and -0.15 <= self.obstacle_y_list[-1] <= 0.15):
                            else:
                                self.is_static = True
                                self.is_dynamic = False

                        continue

                    if self.is_dynamic:
                        if -0.4 < self.obstacle_dynamic_static_y < 0.5 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                            self.publishMotorSteerMsg(0, self.steer_msg)

                            # if self.finish_second_detection == False:
                            #     self.dynamic_obstacle_y_list.append(self.obstacle_dynamic_static_y)

                            #     if len(self.dynamic_obstacle_y_list) >= 400 and self.finish_second_detection == False:
                            #         if abs(self.dynamic_obstacle_y_list[3] - self.dynamic_obstacle_y_list[-3]) <= 0.08:
                            #             self.finish_second_detection = True
                            #             self.is_static = True 
                            #             self.is_dynamic = False


                            self.obstacle_y_list =[]
                            continue

                        elif self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0:
                            self.is_dynamic = False
                            self.obstacle_y_list = []
                            self.finish_detection = False

                        

                    elif self.is_static:
                        if self.finish_detection == True:
                            self.is_goal_arrived = False
                            self.is_static = False
                            self.obstacle_y_list = []
                            continue
                    

                    # 장애물을 지나갔다는 판단
                    if self.obstacle_dynamic_static_x < 0.1 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                        self.finish_detection = False
                        self.finish_second_detection = False
                        self.obstacle_y_list = []
                        self.dynamic_obstacle_y_list = []
                        self.is_dynamic = False
                        self.is_static = False
                    #--------------------------------------------------------------------------------------------------#

                    self.is_slidewindow = True
                

                # ########################################## 3 3 3 3 3 ##########################################
                elif self.stopline_cnt == 3:  

                    #--------------------------------------------- 장애물 ---------------------------------------------#
                    if (0.1 < self.obstacle_dynamic_static_x < 1.5) and self.finish_detection == False:
                        self.publishMotorSteerMsg(0, self.steer_msg)
                        self.obstacle_y_list.append(self.obstacle_dynamic_static_y)

                        # if len(self.obstacle_y_list) >= 10 and self.finish_detection == False:
                        #     if abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]) > 0.08:
                        #         print("y delta: ", abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]))
                        #         # self.obstacle_y_list.sort()
                        #         # self.finish_detection = True
                        #         self.is_dynamic = True
                        #         self.is_static = False
                        #     else:
                        #         print("아직모름 y delta is too small")

                        if len(self.obstacle_y_list) == 400 and self.finish_detection == False :
                            self.finish_detection = True
                            # self.obstacle_y_list.sort()
                
                            if abs(self.obstacle_y_list[100] - self.obstacle_y_list[-3]) > 0.08:
                                self.is_dynamic= True
                                self.is_static = False
                            # elif (-0.15 <= self.obstacle_y_list[0] <= 0.15 and -0.15 <= self.obstacle_y_list[-1] <= 0.15):
                            else:
                                self.is_static = True
                                self.is_dynamic = False
                   
                        continue

                    if self.is_dynamic:
                        if -0.4 < self.obstacle_dynamic_static_y < 0.5 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                            self.publishMotorSteerMsg(0, self.steer_msg)

                            # if self.finish_second_detection == False:
                            #     self.dynamic_obstacle_y_list.append(self.obstacle_dynamic_static_y)

                            #     if len(self.dynamic_obstacle_y_list) >= 400 and self.finish_second_detection == False:
                            #         if abs(self.dynamic_obstacle_y_list[100] - self.dynamic_obstacle_y_list[-3]) <= 0.08:
                            #             self.finish_second_detection = True
                            #             self.is_static = True
                            #             self.is_dynamic = False


                            self.obstacle_y_list =[]
                            continue

                        elif self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0:
                            self.is_dynamic = False
                            self.obstacle_y_list = []
                            self.finish_detection = False

                    
                    if self.is_static:
                        if self.finish_detection == True:
                            self.speed = 4
                            if (-0.25 < self.obstacle_dynamic_static_y < 0.25) and (0.5 < self.obstacle_dynamic_static_x < 1.5): 
                                self.is_static_ob_passed = False
                                if self.reverse_betting_flag == True:
                                    #1 단순 이진화 하는 방법 
                                    if self.status_msg_heading < 70:
                                        self.reverse_betting_pose = 108
                                    else:
                                        self.reverse_betting_pose = 115
                                    self.reverse_betting_flag = False


                            # 일단 피하는 두개 코드 
                            # 오른쪽 -> 왼쪽으로 회피 
                            if self.current_lane == "RIGHT" and self.is_static_ob_passed == False:
                                self.comeback_slidewindow = False
                                self.speed = 1
                                # print("왼쪽으로 바꿀거에요")

                                
                                if (self.reverse_betting_pose <= self.status_msg_heading):

                                    self.current_lane = "LEFT"
                                    self.is_static_ob_passed = True
                                    self.curved_index_at_static = 320
                                    
                                else:
                                    self.curved_index_at_static  = -320.1
                                    self.center_index = self.curved_index_at_static
                                
                                

                                    # if self.center_index < 100:# 100
                                        
                                    #     self.center_index = 100
                                    

                            # 왼쪽 -> 오른쪽으로 회피 
                            elif self.current_lane == "LEFT" and self.is_static_ob_passed == False:
                                self.speed = 2
                                if (self.status_msg_heading <= 70):
                                    self.is_static_ob_passed =True
                                    self.current_lane = "RIGHT"
                                    self.curved_index_at_static = 320
                                    

                                self.curved_index_at_static += 10
                                self.center_index = self.curved_index_at_static

                                if self.center_index > 540:
                                    self.center_index = 540
                                
                            
                            # 피하고 나서 다시 돌아가는 코드#######################################
                            # 왼쪽 차선에서 차선유지 
                            elif self.current_lane == "LEFT" and self.is_static_ob_passed == True:
                                self.speed = 2
                                
                                if (89 <= self.status_msg_heading <= 91):
                                    # self.is_static = False
                                    self.curved_index_at_static = 320
                                
                                elif self.status_msg_heading < 88:
                                    self.curved_index_at_static -= 1
                                else:
                                    self.curved_index_at_static += 1
                                self.center_index = self.curved_index_at_static

                            # 오른쪽 차선에서 차선 유지 
                            elif self.current_lane == "RIGHT" and self.is_static_ob_passed == True:
                                self.comeback_slidewindow = True
                                
                            else:
                                pass

                            
                            if self.comeback_slidewindow == False:
                                self.standard_line = x//2
                                self.degree_per_pixel = 1/x
                                self.prev_center_index = self.center_index
                                self.steer_msg.data = 0.5 + (self.center_index - self.standard_line) * self.degree_per_pixel
                                # print(self.steer_msg.data)
                                self.motor_msg.data = self.speed * 300
                                self.obstacle_y_list = []
                                self.publishMotorSteerMsg(self.motor_msg, self.steer_msg)
                                continue
                            elif self.comeback_slidewindow == True:
                                self.comeback_slidewindow == False
                                # self.is_static = False
                                self.is_static_ob_passed = True
                                pass
                    

                    # 장애물을 지나갔다는 판단
                    if self.obstacle_dynamic_static_x < 0.1 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                        self.finish_detection = False
                        self.finish_second_detection = False
                        self.obstacle_y_list = []
                        self.dynamic_obstacle_y_list = []
                        self.is_dynamic = False
                        self.is_static = False
                    #--------------------------------------------------------------------------------------------------#
                        
                    self.is_slidewindow = True

                # ########################################## 4 4 4 4 4 ##########################################
                elif self.stopline_cnt == 4:
                    self.sliding_window_select_line = 'Right'

                    if self.status_msg_x >= 12 and self.status_msg_y < 4.7:
                        self.sliding_window_select_line = 'Right'
                        #--------------------------------------------- 장애물 ---------------------------------------------#
                        if (0.1 < self.obstacle_dynamic_static_x < 1.5) and self.finish_detection == False:
                            self.publishMotorSteerMsg(0, self.steer_msg)
                            self.obstacle_y_list.append(self.obstacle_dynamic_static_y)

                            # if len(self.obstacle_y_list) >= 10 and self.finish_detection == False:
                            #     if abs(self.obstacle_y_list[3] - self.obstacle_y_list[-3]) > 0.08:

                            #         self.is_dynamic = True
                            #         self.is_static = False
                            
                            if len(self.obstacle_y_list) == 400 and self.finish_detection == False :
                                self.finish_detection = True
                    
                                if abs(self.obstacle_y_list[100] - self.obstacle_y_list[-3]) > 0.08:
                                    self.is_dynamic = True
                                    self.is_static = False
                                # elif (-0.15 <= self.obstacle_y_list[0] <= 0.15 and -0.15 <= self.obstacle_y_list[-1] <= 0.15):
                                else:
                                    self.is_static = True
                                    self.is_dynamic = False

                            continue

                        if self.is_dynamic:
                            if -0.4 < self.obstacle_dynamic_static_y < 0.5 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                                self.publishMotorSteerMsg(0, self.steer_msg)

                                # if self.finish_second_detection == False:
                                #     self.dynamic_obstacle_y_list.append(self.obstacle_dynamic_static_y)

                                #     if len(self.dynamic_obstacle_y_list) >= 400 and self.finish_second_detection == False:
                                #         if abs(self.dynamic_obstacle_y_list[100] - self.dynamic_obstacle_y_list[-3]) <= 0.08:
                                #             self.finish_second_detection = True
                                #             self.is_static = True
                                #             self.is_dynamic = False


                                self.obstacle_y_list =[]
                                continue

                            elif self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0:
                                self.is_dynamic = False
                                self.obstacle_y_list = []
                                self.finish_detection = False

                    
                        if self.is_static:
                            if self.finish_detection == True:
                                self.speed = 2
                                if self.status_msg_y <= 3.1 or self.is_on_laneswitching == True:
                                    # static passed : True : 안피해도 된다
                                    #                 False: 피해야한다. 

                                    # 장애물이 현재 같은 선상에 있을 경우 
                                    if (-0.12 < self.obstacle_dynamic_static_y < 0.12) and (0.5 < self.obstacle_dynamic_static_x < 1.5):
                                        self.is_static_ob_passed = False


                                    # 일단 피하는 두개 코드 
                                    # 오른쪽 -> 왼쪽으로 회피 
                                    if self.current_lane == "RIGHT" and self.is_static_ob_passed == False:
                                        self.is_on_laneswitching = True
                                        if (114 <= self.status_msg_heading):
                                            self.current_lane = "LEFT"
                                            self.is_static_ob_passed = True
                                            self.curved_index_at_static = 320
                                            
                                        else:
                                            self.curved_index_at_static -= 10
                                            self.center_index = self.curved_index_at_static

                                            if self.center_index < 100:
                                                
                                                self.center_index = 100
                                            

                                    # 왼쪽 -> 오른쪽으로 회피 
                                    elif self.current_lane == "LEFT" and self.is_static_ob_passed == False:
                                        self.is_on_laneswitching = True
                                        if (self.status_msg_heading <= 70):
                                            self.is_on_laneswitching = False
                                            self.is_static_ob_passed =True
                                            self.current_lane = "RIGHT"
                                            self.curved_index_at_static = 320
                                            

                                        self.curved_index_at_static += 10
                                        self.center_index = self.curved_index_at_static

                                        if self.center_index > 540:
                                            self.center_index = 540
                                        
                                    
                                    # 피하고 나서 다시 돌아가는 코드#######################################
                                    # 왼쪽 차선에서 차선유지 
                                    elif self.current_lane == "LEFT" and self.is_static_ob_passed == True:
                                        self.is_on_laneswitching = True
                                        
                                        if (89 <= self.status_msg_heading <= 91):
                                            self.is_on_laneswitching = False
                                            # self.is_static = False
                                            self.curved_index_at_static = 320
                                        
                                        elif self.status_msg_heading < 88:
                                            self.curved_index_at_static -= 1
                                        else:
                                            self.curved_index_at_static += 1
                                        self.center_index = self.curved_index_at_static

                                    # 오른쪽 차선에서 차선 유지 
                                    elif self.current_lane == "RIGHT" and self.is_static_ob_passed == True:
                                        self.comeback_slidewindow = True
                                        # if (88<= self.status_msg_heading <= 92):
                                            # self.is_static = False
                                            # self.curved_index_at_static = 320
                                        
                                        # elif self.status_msg_heading < 88:
                                            # self.curved_index_at_static -= 3
                                        # else:
                                            # self.curved_index_at_static += 3
                                        # self.center_index = self.curved_index_at_static
                                        
                                    else:
                                        pass
                                
                                

                                elif self.status_msg_y > 3.1 and self.is_on_laneswitching == False:
                                    
                                    # if self.obstacle_dynamic_static_y >= 0.0:
                                    #     self.is_available_to_go = True


                                    if self.current_lane == "LEFT" and self.comeback_right == False:

                                        if self.obstacle_dynamic_static_y <= 0:
                                            self.speed = 4
                                            self.comeback_slidewindow = True
                                            self.comback_right = True


                                        # if self.is_available_to_go == False and (88<= self.status_msg_heading <= 92):
                                        #     # if (88<= self.status_msg_heading <= 92):
                                            
                                        else:

                                            if (self.status_msg_heading <= 70):
                                                self.comeback_right = True
                                                self.current_lane = "RIGHT"
                                                self.curved_index_at_static = 320
                                                

                                            self.curved_index_at_static += 10
                                            self.center_index = self.curved_index_at_static

                                            if self.center_index > 540:
                                                self.center_index = 540


                                    elif self.current_lane == "RIGHT" and self.comeback_right == True:

                                        if (-0.2 < self.obstacle_dynamic_static_y < 0.2) and (0.5 < self.obstacle_dynamic_static_x < 1.5):
                                            if (113 <= self.status_msg_heading):
                                                self.current_lane = "LEFT"
                                                self.is_static_ob_passed = True
                                                self.comeback_right = False
                                                self.curved_index_at_static = 320
                                                
                                            else:
                                                self.curved_index_at_static -= 10
                                                self.center_index = self.curved_index_at_static

                                                if self.center_index < 100:
                                                    
                                                    self.center_index = 100

                                        else:    
                                            self.comeback_slidewindow = True
                                        
                                        # if (89<= self.status_msg_heading <= 95):
                                        #     # self.is_static = False
                                        #     self.curved_index_at_static = 320
                                        #     self.comeback_slidewindow = True
                                        
                                        # elif self.status_msg_heading < 88:
                                        #     self.curved_index_at_static -= 3
                                        # else:
                                        #     self.curved_index_at_static += 3
                                        # self.center_index = self.curved_index_at_static

                                    elif self.current_lane == 'RIGHT' and self.comeback_right == False:
                                        self.comeback_right = True

                                
                                if self.comeback_slidewindow == False:
                                    self.standard_line = x//2
                                    self.degree_per_pixel = 1/x
                                    self.prev_center_index = self.center_index
                                    self.steer_msg.data = 0.5 + (self.center_index - self.standard_line) * self.degree_per_pixel
                                    self.motor_msg.data = self.speed * 300
                                    self.obstacle_y_list = []
                                    self.publishMotorSteerMsg(self.motor_msg, self.steer_msg)
                                    continue
                                elif self.comeback_slidewindow == True:
                                    self.comeback_slidewindow = False
                                    # self.is_static = False
                                    self.is_static_ob_passed = True
                                    pass


                            
                            
                        

                        # 장애물을 지나갔다는 판단
                        if self.obstacle_dynamic_static_x < 0.1 and not (self.obstacle_dynamic_static_x == 0 and self.obstacle_dynamic_static_y == 0):
                            self.finish_detection = False
                            self.finish_second_detection = False
                            self.obstacle_y_list = []
                            self.dynamic_obstacle_y_list = []
                            self.is_dynamic = False
                            self.is_static = False
                        #--------------------------------------------------------------------------------------------------#

                    if self.status_msg_x >= 12:
                        self.is_slidewindow = True
                        # self.speed = 8
                        # print(self.current_lane)
                        if self.current_lane == "LEFT":
                            self.speed = 4
                        else: 
                            self.speed = 6
                    else:
                        self.is_slidewindow = False
                        self.speed = 4 # 4

                        if self.status_msg_y > 4.3: # 4.5
                            try:
                                if len(self.left_indices) != 0 and len(self.right_indices) != 0:
                                # both line
                                    center_left_index = (self.left_indices[0] + self.right_indices[0])//2
                                    self.center_index = center_left_index

                                
                                # left line
                                elif len(self.left_indices) != 0 and len(self.right_indices) == 0:
                                    center_left_index = (self.left_indices[0] + self.left_indices[-1])//2
                                    self.center_index = center_left_index + 95 # 90

                                
                                # right line
                                elif len(self.left_indices) == 0 and len(self.right_indices) != 0:
                                    center_right_index = (self.right_indices[0] + self.right_indices[-1])//2
                                    self.center_index = center_right_index - 480

                        
                                # no line
                                else:
                                    self.center_index = 135
                            except:         # 양쪽 차선을 다 보되 한쪽 차선만 보이면 그 차선은 무조건 바깥쪽 차선으로 간주
                                self.center_index = self.prev_center_index
                        
                        elif self.status_msg_y <= 4.3:
                            self.is_slidewindow = True
                

                # # 로터리 하면서 바로 왼쪽으로 나가는 버전 
                # ########################################## 5 5 5 5 5  ##########################################
                elif self.stopline_cnt == 5:
                    
                    self.is_slidewindow = False

                    self.speed = 2
                    
                    self.up_hist_end_line = y//8*7
                    self.down_hist_start_line = y//8*7

                    if self.rotary_brake_flag == False:
                        self.brakeWithTime(1)
                        self.rotary_brake_flag = True

                    elif self.rotary_brake_flag == True:
                        
                        if self.is_rotary_entered == False:
                            if self.obstacle_rotary_x != 0 or self.obstacle_rotary_y != 0:
                                distance_from_rotary_obstacle = sqrt(self.obstacle_rotary_x ** 2 + self.obstacle_rotary_y ** 2)
                                if distance_from_rotary_obstacle < 1.7:  # 0.2
                                    self.speed  = 0
                                else:
                                    self.is_rotary_entered = True
                            else:
                                self.speed = 2
                                self.is_rotary_entered = True

                        elif self.is_rotary_entered == True:
                            # 들어갔음에도 안가는 경우 발생
                            if self.obstacle_rotary_x != 0 or self.obstacle_rotary_y != 0:
                                distance_from_rotary_obstacle = sqrt(self.obstacle_rotary_x ** 2 + self.obstacle_rotary_y ** 2)
                                if distance_from_rotary_obstacle < 0.6 and (-0.25 < self.status_msg_y < 0.25):
                                    self.speed = 0
                                

                            if self.is_curved_finished_at_five == False:
                                # print("커브중")
                                if (-180 <= self.status_msg_heading <= -160):
                                    self.is_curved_finished_at_five = True
                                
                                if self.speed != 0:
                                    self.curved_index_at_five += 5.5

                                self.center_index = self.curved_index_at_five
                                if self.center_index > 545:
                                    self.center_index = 545
                                # print('커브인덱스:', self.center_index) 
                                    
                            #  rotary 이후 교차로 전 
                            # if self.status_msg_x < 7.3:
                            else:
                                # print("교완")
                                try:
                                    # both line
                                    if len(self.left_indices) != 0 and len(self.right_indices) != 0:
                                        self.center_index = (self.left_indices[0] + self.right_indices[-1])//2 +15
                                    
                                    # left line

                                    elif len(self.left_indices) != 0 and len(self.right_indices) == 0:
                                        self.center_index = self.left_indices[-1] + 520
                                    
                                    # right line

                                    elif len(self.left_indices) == 0 and len(self.right_indices) != 0:
                                        self.center_index = self.right_indices[-1] + 10

                                    
                                    # no line
                                    else:
                                        self.center_index = self.prev_center_index
                                except:
                                    self.center_index = 640



                ########################################## 6 6 6 6 6 ##########################################
                elif self.stopline_cnt == 6:

                    self.up_hist_end_line = y//4*3
                    self.down_hist_start_line = y//4*3 

                    if self.is_green_left_light == False:
                        self.is_traffic_passed = False

                    if self.is_green_left_light == False and self.is_traffic_stopped == False:
                        # self.speed = 0
                        self.publishMotorSteerMsg(0, self.steer_msg)
                        continue

                    else:
                        self.is_traffic_stopped = True
                        self.speed = 3.5

                    if self.is_curved_finished_at_six == False and self.is_green_left_light == True:

                        # 신호등에서 멈추지 않고 바로 갔을 경우 (원래 값)
                        if self.is_traffic_passed == True:
                            # print("커브중")
                            if (-100 <= self.status_msg_heading <= -90):
                                self.is_curved_finished_at_six = True
                            
                            # if self.is_traffic_light == False:
                            self.curved_index_at_six -= 4

                            self.center_index = self.curved_index_at_six
                            if self.center_index < 85: # 80
                                self.center_index = 85
                        # 신호등에서 멈춘 후에 진행했을 경우
                        else:
                            # print("커브중")
                            # print(self.center_index)
                            if (-100 <= self.status_msg_heading <= -90):
                                self.is_curved_finished_at_six = True
                            
                            # if self.is_traffic_light == False:
                            self.curved_index_at_six -= 4


                            self.center_index = self.curved_index_at_six
                            if self.center_index < 93:  # 85
                                self.center_index = 93

                    else:
                        try:
                            # both line
                            if len(self.left_indices) != 0 and len(self.right_indices) != 0:
                                self.center_index = self.left_indices[-1] + 155

                            
                            # left line
                            elif len(self.left_indices) != 0 and len(self.right_indices) == 0:
                                # self.center_index = self.left_indices[-1] + 180
                                self.center_index = self.left_indices[0] + 540
                            
                            # right line
                            elif len(self.left_indices) == 0 and len(self.right_indices) != 0:
                                self.center_index = self.right_indices[-1] + 120
                            
                            # no line
                            else:
                                self.center_index = self.prev_center_index

                        except:
                            self.center_index = self.prev_center_index

                    # print('커브인덱스:', self.center_index)

                ########################################## 7 7 7 7 7 ##########################################
                elif self.stopline_cnt == 7:
                    

                    if self.is_curved_finished_at_seven == False:
                        self.speed = 4

                        if (-180 <= self.status_msg_heading <= -170):
                            self.is_curved_finished_at_seven = True
                        
                        self.curved_index_at_seven +=6
                        self.center_index = self.curved_index_at_seven
                        if self.center_index > 555: # 550
                            self.center_index = 555
                        # print('커브인덱스:', self.center_index)

                    else:
                        self.speed = 8 
                        self.is_slidewindow = True
                        self.sliding_window_select_line = 'Left'

                    if self.is_curved_finished_at_seven == True and self.status_msg_y >= -1.4:
                        self.is_slidewindow = False
                        
                        if 89.5 <= self.status_msg_heading <= 90.5:
                            self.center_index = 320
                        elif self.status_msg_heading < 89.5:
                            self.center_index = 319
                        elif 90.5 < self.status_msg_heading :
                            self.center_index = 321
                        else:
                            self.center_index = 320


            

                ########################################## 8 8 8 8 8 ##########################################
                elif self.stopline_cnt == 8:

                    if self.status_msg_y <= 1.4:
                        self.is_slidewindow = False
                        if 89.5 <= self.status_msg_heading <= 90.5:
                            self.center_index = 320
                        elif self.status_msg_heading < 89.5:
                            self.center_index = 319
                        elif 90.5 < self.status_msg_heading :
                            self.center_index = 321
                        else:
                            self.center_index = 320

                    elif self.status_msg_y > 1.4:
                        self.is_slidewindow = True
                        self.sliding_window_select_line = 'Left'
                        # if self.status_msg_x >= 1.2:
                        #     self.center_index = 320


                ########################################## 9 9 9 9 9 ##########################################
                                
                elif self.stopline_cnt == 9:

                    if self.status_msg_x <= 3.0:
                        self.is_slidewindow = False
                        self.center_index = 320

                    else:
                        self.brakeWithTime(1000)


                # ############################################# END #############################################
                else:
                    self.is_slidewindow = True
                    self.sliding_window_select_line = 'Right'
                
                # ############################################# END #############################################
                
            
                try:
                    cv2.rectangle(self.warped_img, [self.left_indices[0], 0], [self.left_indices[-1], y], [255,0,0], 2)
                except:
                    pass
                try:
                    cv2.rectangle(self.warped_img, [self.right_indices[0], 0], [self.right_indices[-1], y], [255,0,0], 2)
                except:
                    pass
                try:
                    cv2.rectangle(self.warped_img, [self.center_index-3, 238], [self.center_index+3, 242], [0,0,255], 3)
                except:
                    pass
                
                # print(f"following index: {self.center_index}")


                self.out_img, self.x_location, _ = self.slidewindow.slidewindow(self.bin_img, self.sliding_window_select_line)

                self.standard_line = x//2
                self.degree_per_pixel = 1/x
                self.prev_center_index = self.center_index
                
                pid = PID(0.090, 0.0000, 0.26)
                angle = pid.pid_control(self.center_index - 320)
    
                if self.is_slidewindow is False:
                    self.steer_msg.data = 0.5 + (self.center_index - self.standard_line) * self.degree_per_pixel
  
                elif self.is_slidewindow is True:
                    self.center_index = self.x_location
                    # print(self.center_index)
                    angle = pid.pid_control(self.center_index - 320)
                    self.steer_msg.data = 0.5 + angle/100
 
                # print(self.steer_msg.data)

    
                self.motor_msg.data = self.speed * 300
                self.publishMotorSteerMsg(self.motor_msg, self.steer_msg)

                # cv2.imshow("out_img", self.out_img)
                # cv2.waitKey(1)

            else:
                # print("SLAM COMMAND")
                self.publishMotorSteerMsg(self.nav_motor_msg, self.nav_steer_msg)

            rate.sleep()



    def cam_CB(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)


    def imu_CB(self, msg):
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z

        self.yaw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        self.status_msg_heading = self.yaw*180/pi


    def traffic_CB(self, msg):
        if msg.trafficLightIndex == "SN000005" and (msg.trafficLightStatus == 33):
            self.is_green_left_light = True
        else: 
            self.is_green_left_light = False
            
        
    
    def obstacle_dynamic_static_CB(self, msg):
        self.obstacle_dynamic_static_x = msg.x
        self.obstacle_dynamic_static_y = msg.y

    
    def obstacle_rotary_CB(self, msg):
        self.obstacle_rotary_x = msg.x
        self.obstacle_rotary_y = msg.y


    def goal_reached_CB(self, msg):
        if msg.status.text == "Goal reached.":
            self.is_goal_arrived = True

            self.finish_detection = False
            self.obstacle_y_list = []
            self.is_dynamic = False
            self.is_static = False

            if self.stopline_cnt == 3:
                self.stopline_cnt = 4


    def nav_motor_CB(self, msg):
        self.nav_motor_msg = msg.data


    def nav_steer_CB(self, msg):
        self.nav_steer_msg = msg.data


    def odomCB(self, msg): 
        self.odom_status.position.x = msg.pose.pose.position.x
        self.odom_status.position.y = msg.pose.pose.position.y
        
        self.status_msg_x = msg.pose.pose.position.x -18.5
        self.status_msg_y = msg.pose.pose.position.y + 4.5

        if self.cnt_odom == 0:
            self.prev_odom.position.x = self.odom_status.position.x
            self.prev_odom.position.y = self.odom_status.position.y
            self.cnt_odom = 1
        # print(f"Odom: {self.odom_status.position.x}")
            

    def check_odom(self):
        # prev, present 차이 계산 
        distance_odom = sqrt(pow(self.odom_status.position.x - self.prev_odom.position.x, 2) + pow(self.odom_status.position.y - self.prev_odom.position.y, 2))

        if distance_odom > 3:
     
            if 0.66 < self.status_msg_x < 4.38 and -5.0 < self.status_msg_y < -3.4:
                self.stopline_cnt = 1  
            elif 5.96 < self.status_msg_x < 9.32 and -5.0 < self.status_msg_y < -3.4:
                self.stopline_cnt = 2
            elif 11.25 < self.status_msg_x < 14.38 and -5.0 < self.status_msg_y < -3.4:
                self.stopline_cnt = 3
          
            elif 14.09 < self.status_msg_x < 15.69 and -1.85 < self.status_msg_y < 0.31:
                self.stopline_cnt = 4

            elif 7.8 < self.status_msg_x < 9.8 and 1.39 < self.status_msg_y < 2.86:
                self.stopline_cnt = 5

            elif 4.4 < self.status_msg_x < 6.2 and -0.3 < self.status_msg_y < 1.7:
                self.stopline_cnt = 6

            elif 3.0 < self.status_msg_x < 4.3 and -3.0 < self.status_msg_y < -1.1:
                self.stopline_cnt = 7

            elif -2.2 < self.status_msg_x < -0.2 and -1.75 < self.status_msg_y < 0.15:
                self.stopline_cnt = 8

            elif 1.55 < self.status_msg_x < 3.0 and 3.8 < self.status_msg_y < 5.8:
                self.stopline_cnt = 9

            else:
                self.stopline_cnt += 1

            # prev 갱신
            self.prev_odom.position.x = self.odom_status.position.x
            self.prev_odom.position.y = self.odom_status.position.y
            

    def brakeWithTime(self, brake_time):
        for i in range(1000 * brake_time):
            self.publishMotorSteerMsg(0, self.steer_msg)
            time.sleep(0.001)


    def rotaryBrakeWithTime(self, curve_time, brake_time):
        for i in range(100000 * curve_time):
            self.publishMotorSteerMsg(4, 1)   
            # time.sleep(0.001)

        for i in range(1000 * brake_time):
            self.publishMotorSteerMsg(0, self.steer_msg)
            time.sleep(0.001)


    def publishMotorSteerMsg(self, motor_msg, steer_msg):
        self.motor_pub.publish(motor_msg)
        self.steer_pub.publish(steer_msg)
    

if __name__ == "__main__":
    try: 
        lane_detection_node = LaneDetection()
    except rospy.ROSInterruptException:
        pass
