#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from darknet_ros.msg import BoundingBoxes
from service_ctl.srv import ugv_move, ugv_moveResponse
from std_srvs.srv import Trigger, TriggerResponse
import tf

import math

class GotoBrick:
    def __init__(self):
        rospy.init_node('goto_brick', anonymous=True)

        self.max_x_speed                 = rospy.get_param('~max_x_speed', 1.5)        
        self.max_y_speed                 = rospy.get_param('~max_y_speed', 1.5)        
        self.max_yaw_speed               = rospy.get_param('~max_yaw_speed', 0.75)        
        self.x_axis_reduce_gain          = rospy.get_param('~x_gain', 0.1)
        self.y_axis_reduce_gain          = rospy.get_param('~y_gain', 0.1)
        self.yaw_reduce_gain             = rospy.get_param('~yaw_gain', 0.4)
        self.service_control             = rospy.get_param('~service_control', True)

        self.raw_x                       = -1.0
        self.raw_y                       = -1.0
        self.raw_yaw                     = -1.0
        self.plate_position              = BoundingBoxes()
        self.destination_x               = 0.0
        self.destination_y               = 0.0
        self.destination_yaw             = 0.0
        self.result_cmd_vel              = Twist()

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/goal_position', PoseStamped, self.pose_callback, queue_size=10)
        rospy.Subscriber('/arm_cam/bounding_boxes', BoundingBoxes, self.bounding_box_callback, queue_size=10)

        self.start_flag = False

        if self.service_control:
            self.service = rospy.Service('ugv_move_local', ugv_move, self.run)
            rospy.spin()

    def bounding_box_callback(self, bounding_msg):
        self.plate_position = bounding_msg

    def pose_callback(self, pose_msg):
        quaternion = (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.raw_yaw = euler[2]
        self.raw_x = pose_msg.pose.position.x
        self.raw_y= pose_msg.pose.position.y

        rospy.logdebug('x:{} y:{} yaw:{}'.format(self.destination_x, self.destination_y, self.destination_yaw))

    def calc_twist(self):
        control_speed = Twist()
        if self.is_arrived():
            control_speed.linear.x = 0.0
            control_speed.linear.y = 0.0
            control_speed.angular.z = 0.0
        elif self.raw_x < 0.0 and self.raw_y < 0.0:
            #point turn
            control_speed.linear.x = 0.0
            control_speed.linear.y = 0.0
            control_speed.angular.z = 10.0 * math.pi / 180 #convert degree/s to radian/s. 
        elif self.raw_x < 0.5:
            control_speed.linear.x = 0.0
            control_speed.linear.y = self.raw_y * self.y_axis_reduce_gain
            control_speed.angular.z = self.raw_yaw * self.yaw_reduce_gain
        else:
            #goto brick
            control_speed.linear.x = self.raw_x * self.x_axis_reduce_gain
            control_speed.linear.y = self.raw_y * self.y_axis_reduce_gain
            control_speed.angular.z = self.raw_yaw * self.yaw_reduce_gain

        self.result_cmd_vel = control_speed
        
    def align_plate(self):
        control_speed = Twist()
        control_speed.linear.x = 0.0
        control_speed.linear.y = 0.0
        control_speed.angular.z = 0.0
        self.result_cmd_vel = control_speed

    def publish_twist(self):
        if self.result_cmd_vel.linear.x > self.max_x_speed:
            self.result_cmd_vel.linear.x = self.max_x_speed
        if self.result_cmd_vel.linear.y > self.max_y_speed:
            self.result_cmd_vel.linear.y = self.max_y_speed
        if self.result_cmd_vel.angular.z > self.max_yaw_speed:
            self.result_cmd_vel.angular.z = self.max_yaw_speed

        self.twist_pub.publish(self.result_cmd_vel)

    def pose_filter(self):
        pass

    def is_arrived(self): #TODO: Fix the arrive condition to plate position
        if 0.0 < self.raw_x < 0.5 and 0.0 < self.raw_y < 0.4:
            return True
        else:
            return False

    def is_near(self):
        distance = math.sqrt(self.raw_x ** 2 + self.raw_y ** 2)
        if distance < 0.5:
            return True
        else:
            return False

    def run(self, req=None):
        rospy.loginfo('Go to brick!')
        r = rospy.Rate(20)
        seq = 0
        while not rospy.is_shutdown():
            if self.is_near():
                self.align_plate()
            else:
                self.calc_twist()
            self.publish_twist()
            if self.is_arrived():
                break
            r.sleep()
            seq += 1
        rospy.loginfo('Brick closed!')

        if self.service_control:
            return ugv_moveResponse(True)

if __name__ == "__main__":
    gotoBrick = GotoBrick()
    if not gotoBrick.service_control:
        gotoBrick.run()
