#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tf
import cv2
import yaml

class TLDetector(object):
    def __init__(self):
        rospy.init_node('transform_tester')

        self.listener = tf.TransformListener()

        now = rospy.Time(0)
        self.listener.waitForTransform("/base_link", "/world", now, rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("/base_link", "/world", now)
        matrix = self.listener.fromTranslationRotation(trans, rot)

        print(trans)
        print("---")
        print(rot)
        print("---")
        print(matrix)
        
        rospy.spin()

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

