#! /usr/bin/env python3

import rospy
import rospkg
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib

class NavigationClient:
    def __init__(self):
        self.client=actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.cnt=0
        # self.goal_list = list()
        
        # self.waypoint_1 = MoveBaseGoal()
        # self.waypoint_1.target_pose.header.frame_id = 'map'
        # self.waypoint_1.target_pose.pose.position.x = 0.0
        # self.waypoint_1.target_pose.pose.position.y = 0.0
        # self.waypoint_1.target_pose.pose.orientation.w = 1.0
        # self.waypoint_1.target_pose.pose.orientation.z = 0.0

        # self.goal_list.append(self.waypoint_1)

        # self.waypoint_2 = MoveBaseGoal()
        # self.waypoint_2.target_pose.header.frame_id = 'map'
        # self.waypoint_2.target_pose.pose.position.x = 0.0
        # self.waypoint_2.target_pose.pose.position.y = 0.0
        # self.waypoint_2.target_pose.pose.orientation.w = 1.0
        # self.waypoint_2.target_pose.pose.orientation.z = 0.0

        # self.goal_list.append(self.waypoint_2)

        # self.sequence = 0

        # self.end_point_index = len(self.goal_list) - 1

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = 18.113283098018922
        self.goal.target_pose.pose.position.y = -10.556551442735735
        self.goal.target_pose.pose.orientation.w = 0.9998991343616567
        self.goal.target_pose.pose.orientation.z = -0.01420285544563412

        self.start_time = rospy.Time.now()

    def run(self):
        if (self.client.get_state() != GoalStatus.ACTIVE):
            self.start_time = rospy.Time.now()
            # self.sequence = (self.sequence+1)%2
            # self.client.send_goal(self.goal_list[self.sequence])

            self.client.send_goal(self.goal)
           
        else:
            # print("노드 종료")
            rospy.signal_shutdown("노드 종료됨")
            if (rospy.Time.now().to_sec() - self.start_time.to_sec()) > 30:
                # print(rospy.Time.now().to_sec() - self.start_time.to_sec())
                self.stop()
               

    def stop(self):
        self.client.cancel_all_goals()


def main():
    rospy.init_node('navigation_client')
    nc = NavigationClient()
    rate = rospy.Rate(10)  # 10

    while not rospy.is_shutdown():
        nc.run()
        rate.sleep()


if __name__=='__main__':
    main()
                
