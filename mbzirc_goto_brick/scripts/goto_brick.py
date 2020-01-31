#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
#TODO: Change service package
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import tf

class GotoBrick:
    def __init__(self):
        rospy.init_node('goto_brick', anonymous=True)
        
        self.default_param = rospy.get_param('default_param', 1)
        self.service_control = rospy.get_param('service_control', True)

        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = 0.0
        self.destination_x = 0.0
        self.destination_y = 0.0
        self.destination_yaw = 0.0
        self.is_arrived = False
        self.result_cmd_vel = Twist()

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/goal_position', Pose, self.pose_callback, queue_size=10)

        #self.start_flag = rospy.Service('add_two_ints', AddTwoInts, self.start_flag)

        rospy.spin()

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

        rospy.loginfo('x:{} y:{} yaw:{}'.format(self.destination_x, self.destination_y, self.destination_yaw))

    def calc_twist(self):
        if not detected():
            #point turn

        else:
            #goto brick

        
    def publish_twist(self):
        self.twist_pub.publish(self.result_cmd_vel)

    def pose_filter(self):
        pass

    def is_arrived(self):
        return False

    def run(self):
        r = rospy.Rate(10)
        is_arrived = False
        while not rospy.is_shutdown:
            if self.is_arrived():
                break
            self.calc_twist()
            self.publish_twist()
            r.sleep()
        #return 'success'

if __name__ == "__main__":
    gotoBrick = GotoBrick()
    gotoBrick.run()