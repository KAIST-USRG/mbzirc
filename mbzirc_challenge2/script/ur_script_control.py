#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ObjectCount
from std_msgs.msg import String

class URScriptControl:
    def __init__(self):
        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.box_callback)
        rospy.Subscriber('darknet_ros/found_object', ObjectCount, self.found_callback)
        self.ur_script_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=10)
        self.image_width = 640
        self.image_height = 480
        self.speed_x = 0
        self.speed_y = 0
        self.pre_speed_x = 0
        self.pre_speed_y = 0
        self.box_count = 0
        self.command = 'stop'

    def create_ur_script_message(self, command, speed_x, speed_y):
        ur_script = String()
        if command == 'stop':
            ur_script.data = "stopl(1.0)"
        else:
            ur_script.data = "speedl([{},{},0,0,0,0], 0.1, 0.05)".format(speed_x, speed_y)
        return ur_script

    def found_callback(self, data):
        self.box_count = data.count
        pub_msg = self.create_ur_script_message('stop', 0.0, 0.0)

        if self.box_count == 0:
            pub_msg = self.create_ur_script_message('stop', 0.0, 0.0)
        else:
            pub_msg = self.create_ur_script_message(self.command, self.speed_x, self.speed_y)
        rospy.loginfo(pub_msg)
        self.ur_script_pub.publish(pub_msg)

    def box_callback(self, data):
        distance_from_center_x = 0
        distance_from_center_y = 0
        center_x = -1
        center_y = -1
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.command = 'stop'
    
        for object_box in data.bounding_boxes:
            if object_box.Class == "scissors" and object_box.probability > 0.3:
                center_x = (object_box.xmax + object_box.xmin) / 2
                center_y = (object_box.ymax + object_box.ymin) / 2
                
                distance_from_center_x = self.image_width/2 - center_x
                distance_from_center_y = self.image_height/2 - center_y

                # speed: m/s
                self.speed_x = float(distance_from_center_x) / 1500.0
                self.speed_y = float(distance_from_center_y) / -1500.0
                self.command = 'cartesian_speed'
                break



if __name__ == "__main__":
    rospy.init_node('ur_script_control', anonymous=True)
    urscript_control = URScriptControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
