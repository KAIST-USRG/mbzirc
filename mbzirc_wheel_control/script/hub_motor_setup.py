#!/usr/bin/env python
import odrive
from odrive.enums import *

print("finding odrive!")
odrv0 = odrive.find_any()

print("odrv0.axis0 setup")
odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 25
odrv0.axis0.motor.config.current_control_bandwidth = 100
odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90
odrv0.axis0.encoder.config.bandwidth = 100
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.1
odrv0.axis0.controller.config.vel_limit = 1000
odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

print("odrv0.axis1 setup")
odrv0.axis1.motor.config.pole_pairs = 15
odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
odrv0.axis1.motor.config.requested_current_range = 25
odrv0.axis1.motor.config.current_control_bandwidth = 100
odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis1.encoder.config.cpr = 90
odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.controller.config.pos_gain = 1
odrv0.axis1.controller.config.vel_gain = 0.02
odrv0.axis1.controller.config.vel_integrator_gain = 0.1
odrv0.axis1.controller.config.vel_limit = 1000
odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

print("odrv1.axis0 setup")
odrv1.axis0.motor.config.pole_pairs = 15
odrv1.axis0.motor.config.resistance_calib_max_voltage = 4
odrv1.axis0.motor.config.requested_current_range = 25
odrv1.axis0.motor.config.current_control_bandwidth = 100
odrv1.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv1.axis0.encoder.config.cpr = 90
odrv1.axis0.encoder.config.bandwidth = 100
odrv1.axis0.controller.config.pos_gain = 1
odrv1.axis0.controller.config.vel_gain = 0.02
odrv1.axis0.controller.config.vel_integrator_gain = 0.1
odrv1.axis0.controller.config.vel_limit = 1000
odrv1.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

print("odrv1.axis1 setup")
odrv1.axis1.motor.config.pole_pairs = 15
odrv1.axis1.motor.config.resistance_calib_max_voltage = 4
odrv1.axis1.motor.config.requested_current_range = 25
odrv1.axis1.motor.config.current_control_bandwidth = 100
odrv1.axis1.encoder.config.mode = ENCODER_MODE_HALL
odrv1.axis1.encoder.config.cpr = 90
odrv1.axis1.encoder.config.bandwidth = 100
odrv1.axis1.controller.config.pos_gain = 1
odrv1.axis1.controller.config.vel_gain = 0.02
odrv1.axis1.controller.config.vel_integrator_gain = 0.1
odrv1.axis1.controller.config.vel_limit = 1000
odrv1.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

odrv0.save_configuration()
odrv1.save_configuration()
try:
    odrv0.reboot()
except:
    pass
try:
    odrv1.reboot()
except:
    print("finish")
