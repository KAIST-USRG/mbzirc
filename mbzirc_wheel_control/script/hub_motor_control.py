#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import odrive
from odrive.enums import *
import time
import math

def twist_cb(data):
    global odrv0, odrv1

    print(odrv0.axis0.error)
    print(odrv0.axis1.error)
    print(odrv1.axis0.error)
    print(odrv1.axis1.error)

    #if odrv0.axis0.error != 0 or odrv0.axis1.error != 0:
    #    odrv0.axis0.error = 0
    #    odrv0.axis1.error = 0
    #    odrv0.save_configuration()
    #    rospy.loginfo("odrv0 error")
    #if odrv1.axis0.error != 0 or odrv1.axis1.error != 0:
    #    odrv1.axis0.error = 0
    #    odrv1.axis1.error = 0
    #    odrv1.save_configuration()
    #    rospy.loginfo("odrv1 error")

    radius = 0.127
    rpm = data.linear.x * 30 / (radius * math.pi)
    rpm_gain = rpm * 3 / 2

    odrv0.axis0.controller.vel_setpoint = rpm_gain
    odrv0.axis1.controller.vel_setpoint = rpm_gain
    odrv1.axis0.controller.vel_setpoint = rpm_gain
    odrv1.axis1.controller.vel_setpoint = rpm_gain

    rospy.loginfo(str(rpm) + ' '\
                  + str(data.linear.x) + ' '\
                  + str(data.linear.y) + ' '\
                  + str(data.angular.z))

def listener():
    rospy.init_node('hub_motor_control', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, twist_cb, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    global odrv0, odrv1 
    odrv0 = odrive.find_any(serial_number='20803592524B')
    odrv1 = odrive.find_any(serial_number='20633588524B')
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    listener()
