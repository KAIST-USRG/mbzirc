#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Bool
from time import sleep
import os, rospkg



class Robot:
    rospy.init_node('Goal_publisher', anonymous=True)
    def __init__(self):
    #    f = open("/home/usrg/catkin_ws/src/navi/script/waypoint.txt", 'r')
        rospack = rospkg.RosPack()      
        f = open(os.path.join(rospack.get_path("navi"),"script/path_record.txt"),'r')
        # f = open("/home/usrg/catkin_ws/src/mbzirc/navi/script/path_record.txt", 'r')
        # print(f.read())
        lines = f.readlines()
        f.close()
        index =0
        self.change = True
        self.Cnt =1
        self.len = len(lines)
        self.M = np.zeros((self.len,7))

        for line in lines:
            value = line.split()
            self.M[index,0] = float(value[0])
            self.M[index,1] = float(value[1])
            self.M[index,2] = float(value[2])
            self.M[index,3] = float(value[3])
            self.M[index,4] = float(value[4])
            self.M[index,5] = float(value[5])
            self.M[index,6] = float(value[6])
            print(self.M[index,0],self.M[index,1])
            index = index+1
            
        rospy.Subscriber("/outdoor_nav/odometry/EKF_estimated", Odometry, self.odom_callback)
        self.pub_pose_array = rospy.Publisher('/PoseArray/waypoint', PoseArray, queue_size=2)

        rospy.spin()
    def odom_callback(self,data):
        pub_goal = rospy.Publisher('/goal_point', PoseStamped, queue_size=1)
        goal = PoseStamped()
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        # if (self.Cnt < self.len):
            # self.prev_x   = self.M[self.Cnt-1,0]
            # self.prev_y   = self.M[self.Cnt-1,1]
            # self.goal_x   = self.M[self.Cnt,0]
            # self.goal_y   = self.M[self.Cnt,1]
            # # self.dist_prev = np.sqrt((self.prev_x-self.current_x)**2+(self.prev_y-self.current_y)**2)
            # self.dist_goal = np.sqrt((self.goal_x-self.current_x)**2+(self.goal_y-self.current_y)**2)          
            # goal.header.frame_id   = "odom"
            # goal.header.stamp = rospy.Time.now()
            # goal.pose.position.x   = self.M[self.Cnt,0]
            # goal.pose.position.y   = self.M[self.Cnt,1]
            # # goal.pose.position.z   = self.M[self.Cnt,2]
            # goal.pose.orientation.x = self.M[self.Cnt,3]
            # goal.pose.orientation.y = self.M[self.Cnt,4]
            # goal.pose.orientation.z = self.M[self.Cnt,5]
            # goal.pose.orientation.w = self.M[self.Cnt,6]
            # if (self.change==True):
            #         sleep(1)
            #         pub_goal.publish(goal)
            #         self.change=False
            # if (self.dist_prev > self.dist_goal):
            # self.Cnt +=1
            #         self.change=True

        waypoint_array = PoseArray()
        wpt_pose = Pose()
        for i in range(self.len):
            wpt_pose.position.x = self.M[i,0]
            wpt_pose.position.y = self.M[i,1]
            wpt_pose.position.z = self.M[i,2]
            # wpt_pose.position.z = 
            wpt_pose.orientation.x = self.M[i,3]
            wpt_pose.orientation.y = self.M[i,4]
            wpt_pose.orientation.z = self.M[i,5]
            wpt_pose.orientation.w = self.M[i,6]
            waypoint_array.poses.append(wpt_pose)
            # i = i+1
            # print(i)
        self.pub_pose_array.publish(waypoint_array)
            

if __name__ == '__main__':
    try:
	  Robot()
    except rospy.ROSInterruptException:
        pass
