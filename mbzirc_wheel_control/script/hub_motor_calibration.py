#!/usr/bin/env python
import odrive
from odrive.enums import *
import time

print("finding odrive!")
odrv0 = odrive.find_any()

print("odrv0 motor calibration")
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv1.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv1.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
time.sleep(10)

odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
odrv1.axis0.motor.config.pre_calibrated = True
odrv1.axis1.motor.config.pre_calibrated = True

print("odrv0 encoder calibration")
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv1.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv1.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
time.sleep(10)

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
odrv1.axis0.encoder.config.pre_calibrated = True
odrv1.axis1.encoder.config.pre_calibrated = True

print("odrv0 loop control setup")
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.axis1.config.startup_closed_loop_control = True
odrv1.axis0.config.startup_closed_loop_control = True
odrv1.axis1.config.startup_closed_loop_control = True

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
