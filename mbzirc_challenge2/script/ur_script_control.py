#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String

class URScriptControl:
    def __init__(self):
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.box_callback)
        self.ur_script_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=10)

    def create_ur_script_message(self, speed_x, speed_y):
        ur_script = String()
        if abs(speed_x) <= 0.0001 and abs(speed_y) <= 0.00001:
            ur_script.data = "stopl(0.5)"
        else:
            ur_script.data = "speedl([{},{},0,0,0,0], 0.5, 0.05)".format(speed_x, speed_y)
        return ur_script

    def box_callback(self, data):
        distance_from_center_x = 0
        distance_from_center_y = 0
        image_width = 640
        image_height = 480
        center_x = -1
        center_y = -1
        speed_x = 0.0
        speed_y = 0.0

        for object_box in data.bounding_boxes:
            if object_box.Class == "scissors" and object_box.probability > 0.3:
                center_x = (object_box.xmax + object_box.xmin) / 2
                center_y = (object_box.ymax + object_box.ymin) / 2
                break

        if center_x != -1 and center_y != -1:
            distance_from_center_x = image_width/2 - center_x
            distance_from_center_y = image_height/2 - center_y

        # speed: m/s
        speed_x = float(distance_from_center_x) / 1500.0
        speed_y = float(distance_from_center_y) / -1500.0
        # if distance_from_center_x > 10:
        #     speed_x = 0.05
        # elif distance_from_center_x < -10:
        #     speed_x = -0.05
        # if distance_from_center_y > 10:
        #     speed_y = -0.05
        # elif distance_from_center_y < -10:
        #     speed_y = 0.05
        rospy.loginfo('center:{} {}'.format(center_x, center_y))
        rospy.loginfo('distance:{} {}'.format(distance_from_center_x, distance_from_center_y))
        rospy.loginfo('speed:{} {}'.format(speed_x, speed_y))
        self.ur_script_pub.publish(self.create_ur_script_message(speed_x, speed_y))

if __name__ == "__main__":
    rospy.init_node('ur_script_control', anonymous=True)
    urscript_control = URScriptControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")