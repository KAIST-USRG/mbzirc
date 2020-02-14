#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from time import sleep

class Robot:
   rospy.init_node('Goal_publisher', anonymous=True)
   def __init__(self):
#       f = open("/home/jay/catkin_ws/src/navi/waypoint.txt", 'r')
       f = open("path_record.txt", 'r')

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
          index = index+1
       rospy.Subscriber("/outdoor_nav/odometry/EKF_estimated", Odometry, self.odom_callback)
       rospy.spin()
   def odom_callback(self,data):
       pub_goal = rospy.Publisher('/goal_point', PoseStamped, queue_size=1)
       goal = PoseStamped()
       self.current_x = data.pose.pose.position.x
       self.current_y = data.pose.pose.position.y
       if (self.Cnt < self.len):
            self.prev_x   = self.M[self.Cnt-1,0]
            self.prev_y   = self.M[self.Cnt-1,1]
            self.goal_x   = self.M[self.Cnt,0]
            self.goal_y   = self.M[self.Cnt,1]
            self.dist_prev = np.sqrt((self.prev_x-self.current_x)**2+(self.prev_y-self.current_y)**2)
            self.dist_goal = np.sqrt((self.goal_x-self.current_x)**2+(self.goal_y-self.current_y)**2)          
            goal.header.frame_id   = "odom"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x   = self.M[self.Cnt,0]
            goal.pose.position.y   = self.M[self.Cnt,1]
            goal.pose.position.z   = self.M[self.Cnt,2]
            goal.pose.orientation.x = self.M[self.Cnt,3]
            goal.pose.orientation.y = self.M[self.Cnt,4]
            goal.pose.orientation.z = self.M[self.Cnt,5]
            goal.pose.orientation.w = self.M[self.Cnt,6]
            if (self.change==True):
                    sleep(1)
                    pub_goal.publish(goal)
                    self.change=False
            if (self.dist_prev > self.dist_goal):
                    self.Cnt +=1
                    self.change=True

if __name__ == '__main__':
    try:
	  Robot()
    except rospy.ROSInterruptException:
        pass
