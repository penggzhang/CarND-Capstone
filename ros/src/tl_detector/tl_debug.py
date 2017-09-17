#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from debug_msgs.msg import TrafficLightMetadata, Rectangle
from cv_bridge import CvBridge
import tf
import cv2
import yaml
import sys
import math
import numpy as np

class TLDebug(object):

    def __init__(self):
        self.debug_tl_image_pub = rospy.Publisher('debug/image_tl', Image, queue_size=1)
        self.debug_tl_metadata_pub = rospy.Publisher('debug/image_tl_metadata', TrafficLightMetadata, queue_size=1)

    def generate_image_msg(self, image):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.encoding = 'bgr8'
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.step = image.shape[1] * 3
        msg.data = image.tostring()
        return msg

    def generate_image_metadata_msg(self, image, rectangle, color):
        msg = TrafficLightMetadata()
        msg.image = self.generate_image_msg(image)
        msg.crop_area = rectangle
        msg.color = color
        return msg

    def generate_rectangle_msg(self, top, left, bottom, right):
        msg = Rectangle()
        msg.top = top
        msg.bottom = bottom
        msg.left = left
        msg.right = right
        return msg


    def draw_image(self, image, distance, x, y):
        cv2.line(image,(x-10,y-10),(x+10,y+10),(255,0,0),3)
        cv2.line(image,(x-10,y+10),(x+10,y-10),(255,0,0),3)

        cv2.line(image,(x-40,y-20),(x+40,y-20),(255,0,0),3)
        cv2.line(image,(x-40,y-20),(x-40,y+130),(255,0,0),3)
        cv2.line(image,(x+40,y-20),(x+40,y+130),(255,0,0),3)
        cv2.line(image,(x-40,y+130),(x+40,y+130),(255,0,0),3)
        
        font = cv2.FONT_HERSHEY_TRIPLEX
        cv2.putText(image, 'Distance Next TL => {}'.format(distance), (10, 30), font, 1, (0, 255, 0))
        cv2.putText(image, '(X, Y) => {} {}'.format(x, y), (10, 60), font, 1, (0, 255, 0))
        # if 0 <= x <= 800 and 0 <= y <= 600 and distance < 200.0:
        #     cv2.putText(image, 'Traffic Light in Range', (10, 90), font, 1, (0, 255, 0))
        # else:
        #     cv2.putText(image, 'No Traffic Light in Image', (10, 90), font, 1, (0, 0, 255))


    def publish_debug_image(self, image, distance, x, y):
        self.draw_image(image, distance, x, y)
        msg_image = self.generate_image_msg(image)
        self.debug_tl_image_pub.publish(msg_image)


    def publish_debug_image_metadata(self, image, color, top, left, bottom, right):
        msg_rectangle = self.generate_rectangle_msg(top, left, bottom, right)
        msg_metadata = self.generate_image_metadata_msg(image, msg_rectangle, color)
        self.debug_tl_metadata_pub.publish(msg_metadata)
