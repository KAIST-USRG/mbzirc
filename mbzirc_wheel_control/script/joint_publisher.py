#!/usr/bin/env python

import math
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from dynamixel_workbench_msgs.srv import DynamixelCommand


class JointPublisher:
    def __init__(self):
        rospy.init_node('joint_publisher', anonymous=True)
        rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
        self.turn_motor = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command', DynamixelCommand)
        self.front_pub = rospy.Publisher('/front/odrive/odrive_joint', 
                JointTrajectoryControllerState, queue_size=1)
        self.back_pub = rospy.Publisher('/back/odrive/odrive_joint', 
                JointTrajectoryControllerState, queue_size=1)
        rospy.Subscriber('/joint_command', JointTrajectoryControllerState, self.joint_callback, queue_size=1)
        rospy.spin()

    def radian2value(self, radian):
        value = -501923 / 180 * 180 / math.pi * radian
        return value

    def turn_dynamixel(self, motor_id, angle):
        try:
            value = self.radian2value(angle)
            resp1 = self.turn_motor('', motor_id, 'Goal_Position', value)
            return resp1.comm_result
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def joint_callback(self, data):
        front_msg = JointTrajectoryControllerState()
        back_msg = JointTrajectoryControllerState()

        front_msg.joint_names.append('fl_caster_r_wheel_joint')
        front_msg.joint_names.append('fr_caster_r_wheel_joint')
        front_msg.desired.velocities.append(data.desired.velocities[0])
        front_msg.desired.velocities.append(data.desired.velocities[6])
        self.front_pub.publish(front_msg)

        back_msg.joint_names.append('bl_caster_r_wheel_joint')
        back_msg.joint_names.append('br_caster_r_wheel_joint')
        back_msg.desired.velocities.append(data.desired.velocities[2])
        back_msg.desired.velocities.append(data.desired.velocities[4])
        self.back_pub.publish(back_msg)

        result1 = self.turn_dynamixel(1, data.desired.positions[1]) 
        result2 = self.turn_dynamixel(2, data.desired.positions[3]) 
        result3 = self.turn_dynamixel(3, data.desired.positions[5]) 
        result4 = self.turn_dynamixel(4, data.desired.positions[7]) 
    
if __name__ == '__main__':
    try:
        JointPublisher()
    except:
        pass
