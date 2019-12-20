#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import odrive
from odrive.enums import *
import time
import math

class HubMotor():
    def __init__(self):
        pass
    def set_gains(self):
        pass
    def set_rpm(self, rpm):
        pass
    def get_battery(self):
        return battery_voltage
    def get_status(self):
        return (rpm)

class SteerMotor():
    def __init__(self):
        pass
    def set_angle(self, degree):
        pass
    def convert_degree2value(self, degree):
        #value = 
        return value
    def get_status(self):
        return (rpm, degree)
        

class MotorModule():
    def __init__(self):
        hub_motor = HubMotor()
        steer_motor = SteerMotor()
    def set_steer_rpm(self, degree, rpm):
        hub_motor.set_rpm(rpm)
        steer_motor.set_angle(degree)
    def get_module_status(self):
        
        return

class WheelController():
    def __init__(self):
        
    def twist_callback(self, data):
        pass
    def 
        
def twist_cb(data):
    global odrv0, odrv1

    rospy.logdebug(odrv0.axis0.error)
    rospy.logdebug(odrv0.axis1.error)
    rospy.logdebug(odrv1.axis0.error)
    rospy.logdebug(odrv1.axis1.error)

    radius = 0.127
    if data.angular.z > 0.3:
        rpm = -50
        rpm_rf = rpm
        rpm_lf = -rpm
        rpm_rb = rpm
        rpm_lb = -rpm
    elif data.angular.z < -0.3:
        rpm = 50
        rpm_rf = rpm
        rpm_lf = -rpm
        rpm_rb = rpm
        rpm_lb = -rpm
    else:
        speed = math.sqrt(data.linear.x**2 + data.linear.y**2)
        if speed < 0.05:
            rpm = 0
            rpm_rf = rpm
            rpm_lf = rpm
            rpm_rb = rpm
            rpm_lb = rpm
        else:
            rpm = math.sqrt(data.linear.x**2 + data.linear.y**2) * 30 / (radius * math.pi)
            rpm_rf = rpm
            rpm_lf = rpm
            rpm_rb = rpm
            rpm_lb = rpm

    rpm_gain_rf = rpm_rf * 3 / 2
    rpm_gain_lf = rpm_lf * 3 / 2
    rpm_gain_rb = rpm_rb * 3 / 2
    rpm_gain_lb = rpm_lb * 3 / 2

    odrv0.axis0.controller.vel_setpoint = rpm_gain_rf
    odrv0.axis1.controller.vel_setpoint = rpm_gain_lf
    odrv1.axis0.controller.vel_setpoint = rpm_gain_lb
    odrv1.axis1.controller.vel_setpoint = rpm_gain_rb

    rospy.logdebug(str(rpm) + ' '\
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
    rospy.loginfo("20803592524B odrv0 connected!")
    odrv1 = odrive.find_any(serial_number='20633588524B')
    rospy.loginfo("20633588524B odrv1 connected!")
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    listener()
