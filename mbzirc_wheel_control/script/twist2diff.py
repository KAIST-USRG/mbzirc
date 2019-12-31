#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import odrive
from odrive.enums import *
import time
import math

def twist_cb(cmd_vel):
    global odrv0
    ROBOT_WIDTH = 0.560

    rospy.logdebug(odrv0.axis0.error)
    rospy.logdebug(odrv0.axis1.error)

    left_rpm = (cmd_vel.linear.x - cmd_vel.angular.z*ROBOT_WIDTH/2) / (math.pi*0.254) * 120
    right_rpm = (cmd_vel.linear.x + cmd_vel.angular.z*ROBOT_WIDTH/2) / (math.pi*0.254) * 120

    odrv0.axis0.controller.vel_setpoint = left_rpm
    odrv0.axis1.controller.vel_setpoint = right_rpm 

    rospy.logdebug(str(left_rpm) + ' '\
                  + str(right_rpm) + ' '\
                  + str(cmd_vel.linear.x) + ' '\
                  + str(cmd_vel.linear.y) + ' '\
                  + str(cmd_vel.angular.z))

def listener():
    rospy.init_node('diff_robot_control', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, twist_cb, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    global odrv0
    odrv0 = odrive.find_any()
    rospy.loginfo("Connected!!")
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    listener()
