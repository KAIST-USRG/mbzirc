import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
#TODO: Change service package
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse

class GotoBrick:
    def __init__(self):
        rospy.init_node('goto_brick', anonymous=True)
        
        default_param = rospy.get_param('default_param', 1)
        service_control = rospy.get_param('service_control', True)

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/brick_pose', PoseStamped, self.pose_callback, queue_size=10)

        start_flag = rospy.Service('add_two_ints', AddTwoInts, self.start_flag)

    def handle_add_two_ints(self, service_msg):
        print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
        return AddTwoIntsResponse(req.a + req.b)

    def publish_twist(self):
        result_cmd_vel = self.calc_twist()
        self.twist_pub.publish(result_cmd_vel)

    def pose_callback(self, pose_msg):
        pass

    def calc_twist(self):
        pass

    def pose_filter(self):
        pass

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.value)
            r.sleep()

if __name__ == "__main__":
    gotoBrick = GotoBrick()
    gotoBrick.run()