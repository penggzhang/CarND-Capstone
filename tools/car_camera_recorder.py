#! /usr/bin/python
### I got the reference code from https://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (800,600))

def image_callback(msg):
    #print("Received an image!")
    #print(msg.encoding)
    #print(msg.height)
    #print(msg.width)
    try:
        img = bridge.imgmsg_to_cv2(msg, "8UC3")
    except CvBridgeError, e:
        print(e)
    else:
        # Save image as a frame in a video
	out.write(img)

def main():
    print("Initializing ROS Image Listener Node...")
    rospy.init_node('image_listener')

    print("Subscribing to Camera Topic...")
    image_topic = "/camera/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    print("Recording Car Camera to Output video...")

    print("Press ctrl + c to exit...")
    rospy.spin()
    
    out.release()

if __name__ == '__main__':
    main()
