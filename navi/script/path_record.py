#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import numpy as np

class Robot:
  def __init__(self):
     rospy.init_node('listener', anonymous=True)
     self.x_prev = 0
     self.y_prev = 0
     self.z_prev = 0
     self.o_x_prev = 0
     self.o_y_prev = 0
     self.o_z_prev = 0
     self.o_w_prev = 1
     rospy.Subscriber("/outdoor_nav/odometry/EKF_estimated", Odometry, self.callback)
     rospy.spin()
  def callback(self,data):
     if (np.sqrt((data.pose.pose.position.x-self.x_prev)**2 + (data.pose.pose.position.y-self.y_prev)**2) > 5):
         f = open("path_record.txt",'a')
         st = str(data.pose.pose.position.x) + ' ' + str(data.pose.pose.position.y) + ' ' + str(data.pose.pose.position.z) + ' ' + str(data.pose.pose.orientation.x) + ' ' + str(data.pose.pose.orientation.y) + ' ' + str(data.pose.pose.orientation.z)+' ' + str(data.pose.pose.orientation.w) +'\n'
         f.write(st)
         f.close()
         print(1)
         self.x_prev =data.pose.pose.position.x
         self.y_prev =data.pose.pose.position.y
         self.z_prev =data.pose.pose.position.z
         self.o_x_prev =data.pose.pose.orientation.x
         self.o_y_prev =data.pose.pose.orientation.y
         self.o_z_prev =data.pose.pose.orientation.z
         self.o_w_prev =data.pose.pose.orientation.w

    


if __name__ == '__main__':
    try:
	Robot()
    except rospy.ROSInterruptException:
        pass
