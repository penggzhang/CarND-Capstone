#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import sys
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3

VISIBLE_DISTANCE = 300

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.all_light_wps = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and 
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        #The closest waypoint to car
        self.car_position = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_distance_waypoint_to_pose(self, waypoint, pose):
        """Calculate the distance between the waypoint and the pose

        Args:
            waypoint (Pose): waypoint
            pose     (Pose): position to match a waypoint to

        Returns:
            float: distance between waypoint and pose

        """

        diff_x = waypoint.x - pose.x
        diff_y = waypoint.y - pose.y
        diff_z = waypoint.z - pose.z

        return math.sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z)


    def get_distance_btw_two_poses(self, pose_1, pose_2):
        """Calculate the distance between two poses

        Args:
            pose_1 (Pose): pose of 1st point
            pose_2 (Pose): pose of 2nd point

        Returns:
            float: distance between two poses

        """

        diff_x = pose_1.position.x - pose_2.position.x
        diff_y = pose_1.position.y - pose_2.position.y
        diff_z = pose_1.position.z - pose_2.position.z

        return math.sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z)


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        # Brute Force Searching (TODO: Try to reduce the number of operations by using divide and conquer)
        # http://rosettacode.org/wiki/Closest-pair_problem#Python
        min_distance           = sys.maxsize
        nearest_waypoint_index = -1
        #rospy.loginfo("Type of self.waypoints: %s", type(self.waypoints))

        if self.waypoints != None:
            for i in range(0, len(self.waypoints.waypoints)):
                waypoint  = self.waypoints.waypoints[i].pose.pose
                posepoint = pose
                distance = self.get_distance_btw_two_poses(waypoint, posepoint)
                if distance < min_distance:
                    min_distance = distance
                    nearest_waypoint_index = i

        return nearest_waypoint_index


    def get_all_light_wps(self, light_positions):
        """Find the closest waypoint for each traffic light

        Args:
            light_positions: list of 2D (x, y) position of all traffic lights

        Returns:
            all_light_wps: list of waypoint indices

        """
        all_light_wps = []
        pose = Pose()

        for i in range(len(light_positions)):
            pose.position.x = light_positions[i][0]
            pose.position.y = light_positions[i][1]

            wp = self.get_closest_waypoint(pose)
            all_light_wps.append(wp)

        rospy.loginfo("Waypoint indices of all lights: %s", all_light_wps)
        return all_light_wps


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        #Current focal lengths are probably wrong, which leads incorrect pixel transformation.
        #https://discussions.udacity.com/t/focal-length-wrong/358568

        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        x, y = None, None

        # get transform between pose of camera and world frame
        trans, rot = None, None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link", "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link", "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #### Use tranform and rotation to calculate 2D position of light in image
        # http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Transformer.html
        # https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/tf_lab/html/tf.listener.TransformerROS-class.html
        # http://wiki.ros.org/tf/TfUsingPython
        # http://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf
        # http://www.cse.psu.edu/~rtc12/CSE486/lecture13.pdf
        # https://stackoverflow.com/questions/5288536/how-to-change-3d-point-to-2d-pixel-location?rq=1
        # ex. trans = [-1230.0457257142773, -1080.1731777599543, -0.10696510000000001]
        # ex. rot   = [0.0, 0.0, -0.0436201197059201, 0.9990481896069084]
        # ex. matrix = [[  9.96194570e-01   8.71572032e-02   0.00000000e+00  -1.23004573e+03]
        #               [ -8.71572032e-02   9.96194570e-01   0.00000000e+00  -1.08017318e+03]
        #               [  0.00000000e+00   0.00000000e+00   1.00000000e+00  -1.06965100e-01]
        #               [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
        
        #### Forward Projection
        #TODO: Debug rot referenced before assignment
        # World to Camera Transformation (Rigid transformation = rotation + translation)
        if trans != None and rot != None:
            transformation_matrix = self.listener.fromTranslationRotation(trans, rot)
            point_in_world_vector = np.array([[point_in_world.x], [point_in_world.y], [point_in_world.z], [1.0]], dtype=float)
            camera_point = np.dot(transformation_matrix, point_in_world_vector)

            #rospy.loginfo("Point in camera coords: %s", camera_point)

            # Perspective Correction
            #Instead of using the focal lengths in 'sim_traffic_light_config.yaml',
            #experiment by hard coding with some trial values in correct magnitude.
            #TODO: wait course teaching asisstant to classify on focal lengths
            #and then locate the crop window.
            fx, fy = 1200, 800
            x = int(-fx * camera_point[1] / camera_point[0] + image_width  / 2)
            y = int(-fy * camera_point[2] / camera_point[0] + image_height / 2)

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)
        #rospy.loginfo("Projected point: (%s, %s)", x, y)
        
        #TODO use light location to zoom in on traffic light in image

        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        return TrafficLight.UNKNOWN

    def get_upcoming_light_wp(self, car_position, all_light_wps):
        """Find the waypoint of the upcoming light

        Args:
            car_position    (Int): the closest waypoint to the car
            all_light_wps ([Int]): list of the closest waypoint for each light

        Returns: 
            int: the waypoint index of the upcoming light
            int: the index of the upcoming light in [lights] 

        """
        #Find the interval in which the car is
        interval = 0
        if car_position == 0:
            pass
        else:
            for i in range(len(all_light_wps)):
                if car_position <= all_light_wps[i]:
                    interval = i
                    break
        #rospy.loginfo("interval: %s", interval)

        #Find the upcoming light waypoint and index
        #Note: only go one way along an ascending sequence of waypoints
        light_wp = all_light_wps[interval]
        light_id = interval

        return light_wp, light_id


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = self.config['light_positions']
        if(self.pose):
            position_tmp = self.get_closest_waypoint(self.pose.pose)
            if position_tmp != -1:
                self.car_position = position_tmp

        #TODO find the closest visible traffic light (if one exists)

        #Find the closest waypoint for each traffic light
        if self.all_light_wps == None and self.waypoints != None:
            self.all_light_wps = self.get_all_light_wps(self.config['light_positions'])

        #Find the waypoint and index of the upcoming light
        if self.all_light_wps != None and self.car_position != None:
            light_wp, light_id = self.get_upcoming_light_wp(self.car_position, self.all_light_wps)
            #rospy.loginfo("Upcoming light waypoint and index: %s, %s", light_wp, light_id)

            #Find the distance from the car to the upcoming light
            if self.lights != None and self.pose != None:
                light = self.lights[light_id]
                distance_to_light = self.get_distance_btw_two_poses(self.pose.pose, light.pose.pose)
                #rospy.loginfo("Distance to upcoming light: %s", distance_to_light)

                #If the car is farther than a pre-set visible distance,
                #then return -1 for light waypoint and UNKNOWN for light state.
                #Otherwise, proceed to recognize the light state.
                if distance_to_light > VISIBLE_DISTANCE:
                    return -1, TrafficLight.UNKNOWN
                else:
                    state = self.get_light_state(light)

                    #Warn if predicted state conflicts with truth state
                    #if state != light.state:
                    #	rospy.logwarn("Predicted state: %s, Truth state: %s", state, light.state)

                    return light_wp, state
            
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
