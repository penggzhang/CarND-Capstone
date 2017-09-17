#! /usr/bin/python
### I got the reference code from https://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/

import argparse

parser = argparse.ArgumentParser(description='CarND Capstone Car Camera Video Recorder')
parser.add_argument('--mode', type=str, required=True, help='CAMERA or DEBUG (camera with debug information)')
args = parser.parse_args()

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (800,600))

def image_callback(msg):
    #print("Received an image!")
    #print(msg.encoding, msg.height, msg.width)
    try:
        img = bridge.imgmsg_to_cv2(msg, "rgb8")
	img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    except CvBridgeError, e:
        print(e)
    else:
        # Save image as a frame in a video
	out.write(img)

def main():
    print("Initializing ROS Image Listener Node...")
    rospy.init_node('car_camera_recorder')

    print("Subscribing to Car Camera Topic...")

    if args.mode == "CAMERA":
        image_topic = "/image_color"
    else:
        image_topic = "/debug/image_tl"

    rospy.Subscriber(image_topic, Image, image_callback)

    print("Recording Car Camera to Output video...")

    print("Press ctrl + c to exit...")
    rospy.spin()
    
    out.release()

if __name__ == '__main__':
    main()
