#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
#TODO: Change service package
#from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import tf

import math

class GotoBrick:
    def __init__(self):
        rospy.init_node('goto_brick', anonymous=True)
        
        self.x_axis_reduce_gain          = rospy.get_param('~x_gain', 0.2)
        self.y_axis_reduce_gain          = rospy.get_param('~y_gain', 0.2)
        self.service_control             = rospy.get_param('~service_control', True)

        self.raw_x                       = -1.0
        self.raw_y                       = -1.0
        self.raw_yaw                     = -1.0
        self.destination_x               = 0.0
        self.destination_y               = 0.0
        self.destination_yaw             = 0.0
        self.result_cmd_vel              = Twist()

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/goal_position', PoseStamped, self.pose_callback, queue_size=10)

        #self.start_flag = rospy.Service('add_two_ints', AddTwoInts, self.start_flag)

    def handle_add_two_ints(self, service_msg):
        rospy.loginfo('Service received!')
        result = self.run()
        return AddTwoIntsResponse(req.a + req.b)

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
        if 0.0 < self.raw_x < 0.7 and 0.0 < self.raw_y < 0.5:
            control_speed.linear.x = 0.0
            control_speed.linear.y = 0.0
            control_speed.angular.z = 0.0
        elif self.raw_x < 0.0 and self.raw_y < 0.0:
            #point turn
            control_speed.linear.x = 0.0
            control_speed.linear.y = 0.0
            control_speed.angular.z = 10.0 * math.pi / 180 #convert degree/s to radian/s. 
        else:
            #goto brick
            control_speed.linear.x = self.raw_x * self.x_axis_reduce_gain
            control_speed.linear.y = self.raw_y * self.y_axis_reduce_gain
            control_speed.angular.z = 0.0

        self.result_cmd_vel = control_speed
        
    def publish_twist(self):
        self.twist_pub.publish(self.result_cmd_vel)

    def pose_filter(self):
        pass

    def is_arrived(self):
        if 0.0 < self.raw_x < 0.7 and 0.0 < self.raw_y < 0.5:
            rospy.loginfo('Close!!')
            return True
        else:
            return False

    def run(self):
        r = rospy.Rate(20)
        is_arrived = False
        seq = 0
        while not rospy.is_shutdown():
            self.calc_twist()
            self.publish_twist()
            if self.is_arrived():
                break
            r.sleep()
            seq += 1
        #return 'success'

if __name__ == "__main__":
    gotoBrick = GotoBrick()
    gotoBrick.run()
