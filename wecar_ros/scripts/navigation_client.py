#! /usr/bin/env python3

import rospy
import rospkg
from std_msgs.msg import Bool, Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib

class NavigationClient:
    def __init__(self):
        self.stopline = None
        rospy.Subscriber("/stopline_cnt", Int32, self.stopline_CB)

        self.client=actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.cnt=0
        self.goal_list = list()

        self.stop_flag_0 = False
        self.stop_flag_1 = False
        self.stop_flag_2 = False
        self.stop_flag_3 = False


        self.sequence = 0

        self.end_point_index = len(self.goal_list) - 1

        self.goal_0 = MoveBaseGoal()
        self.goal_0.target_pose.header.frame_id = 'map'
        self.goal_0.target_pose.pose.position.x = 17.009463346895896 #16.009463346895896
        self.goal_0.target_pose.pose.position.y = -9.798660786866374 # -10.553446397817408      # -10.543390100139973
        self.goal_0.target_pose.pose.orientation.w = 0.999772302787841 # 0.9998798796801283    # 0.999881303411936
        self.goal_0.target_pose.pose.orientation.z = -0.02133875765590587 # -0.015499232589139468 # -0.015407111580302366
        self.goal_list.append(self.goal_0)

        self.goal_1 = MoveBaseGoal()
        self.goal_1.target_pose.header.frame_id = 'map'
        self.goal_1.target_pose.pose.position.x = 33.4001540305468  # 33.3501540305468
        self.goal_1.target_pose.pose.position.y = -9.810584023996787 # -10.683526076943182
        self.goal_1.target_pose.pose.orientation.w = 0.9999982021898778 # 0.9999948220820374
        self.goal_1.target_pose.pose.orientation.z = 0.0018962112256433554 # 0.0032180442996367488
        self.goal_list.append(self.goal_1)


        # self.goal_2 = MoveBaseGoal()
        # self.goal_2.target_pose.header.frame_id = 'map'
        # self.goal_2.target_pose.pose.position.x = 35.25576989807332 # 37.21576989807332
        # self.goal_2.target_pose.pose.position.y = -1.452630900457581 # -1.9510466300457487
        # self.goal_2.target_pose.pose.orientation.w = 0.7144000929277918 # 0.664689725633568
        # self.goal_2.target_pose.pose.orientation.z = 0.6997374559252652 # 0.747119514292842
        # self.goal_list.append(self.goal_2)



        # self.goal_3 = MoveBaseGoal()
        # self.goal_3.target_pose.header.frame_id = 'map'
        # self.goal_3.target_pose.pose.position.x = 19.844983202022526
        # self.goal_3.target_pose.pose.position.y = -3.9458939473730426
        # self.goal_3.target_pose.pose.orientation.w = 0.708472464861472
        # self.goal_3.target_pose.pose.orientation.z = 0.705738454764306
        # self.goal_list.append(self.goal_3)


        self.start_time = rospy.Time.now()

    def run(self):
        print(self.stopline)

        if self.stopline is None:
            if self.stop_flag_0 == False:
                self.stop()
                self.stop_flag_0 = True
            self.sequence = 0

        elif 0 <= self.stopline <= 2:
            if self.stop_flag_1 == False:
                self.stop()
                self.stop_flag_1 = True
            self.sequence = 1

        # elif 3 <= self.stopline <= 4:
        #     if self.stop_flag_2 == False:
        #         self.stop()
        #         self.stop_flag_2 = True
        #     self.sequence = 2

        # elif self.stopline == 8:
        #     if self.stop_flag_3 == False:
        #         self.stop()
        #         self.stop_flag_3 = True
        #     self.sequence = 3

        if (self.client.get_state() != GoalStatus.ACTIVE):
            self.start_time = rospy.Time.now()

            self.client.send_goal(self.goal_list[self.sequence])

           
        # else:
        #     # print("노드 종료")
        #     # rospy.signal_shutdown("노드 종료됨")
        #     # if (rospy.Time.now().to_sec() - self.start_time.to_sec()) > 30:
        #     #     # print(rospy.Time.now().to_sec() - self.start_time.to_sec())
        #     #     self.stop()
        #     pass
               

    def stop(self):
        self.client.cancel_all_goals()

    def stopline_CB(self, msg):
        self.stopline = msg.data



def main():
    rospy.init_node('navigation_client')
    nc = NavigationClient()
    rate = rospy.Rate(10)  # 10

    while not rospy.is_shutdown():
        nc.run()
        rate.sleep()


if __name__=='__main__':
    main()
                
