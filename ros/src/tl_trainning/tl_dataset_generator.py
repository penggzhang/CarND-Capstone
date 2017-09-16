#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from debug_msgs.msg import TrafficLightMetadata

import cv2
import sys
import math
import numpy as np

class TLDatasetGenerator(object):

    def __init__(self):
        rospy.init_node('tl_dataset_generator')

        sub1 = rospy.Subscriber('debug/image_tl_metadata', TrafficLightMetadata, self.process_metadata)

        self.bridge = CvBridge()

        self.counter = 0

        rospy.spin()

    def process_metadata(self, msg):
        print("metadata received!!!")

        image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        area  = msg.crop_area
        color = msg.color

        crop_image = image[area.top:area.bottom, area.left:area.right]

        cv2.imwrite('./dataset/tl_{}.{}.png'.format(self.counter, color), crop_image)

        self.counter += 1

if __name__ == '__main__':
    try:
        TLDatasetGenerator()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic light dataset generator node.')
