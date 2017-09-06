#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 5       # Number of waypoints published
SPEED_MPH     = 10      # Forward speed in miles per hour
MPH2MPS       = 0.44704 # Conversion miles per hour to meters per second
FRAME_ID      = 'WPT'   # ROS TF Frame ID of published waypoints
DEBUG         = False   # True = Print Statements appear in Terminal with Debug info

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pos_x         = 0.0;
        self.pos_y         = 0.0;
        self.pos_z         = 0.0;
        self.waypoints     = None;
        self.wpt_ahead_idx = 0;       
        self.wpt_ahead     = None;
        self.final_wpts    = None;

        rospy.spin()
        
    def loop(self): 
        # Fill the final waypoint list with all the waypoints ahead
        if self.waypoints != None:
            self.find_next_waypoint()
            self.final_wpts = Lane()
            
            if DEBUG:
                print('WAYPOINT UPDATER :: WPT Ahead ',"x: ",self.wpt_ahead.pose.pose.position.x,"y: ",self.wpt_ahead.pose.pose.position.y,"idx: ",self.wpt_ahead_idx)
            #Form final waypoint list from starting waypoint to waypoints ahead
            final_wpt_idx = self.wpt_ahead_idx+LOOKAHEAD_WPS
            if final_wpt_idx < len(self.waypoints):
                self.final_wpts.waypoints = self.waypoints[self.wpt_ahead_idx:final_wpt_idx]
            else:
                self.final_wpts.waypoints = self.waypoints[self.wpt_ahead_idx:len(self.waypoints)]
            #Fill Speed
            for wpt in self.final_wpts.waypoints:
                wpt.twist.twist.linear.x = 20*MPH2MPS
                wpt.twist.twist.linear.y = 0
                wpt.twist.twist.linear.z = 0
            if DEBUG:
                print('WAYPOINT UPDATER :: Final WPT List Length: ', len(self.final_wpts.waypoints))    
            #Fill Header
            self.final_wpts.header.stamp    = rospy.Time.now()
            self.final_wpts.header.frame_id = FRAME_ID
            #Publish final waypoints
            self.final_waypoints_pub.publish(self.final_wpts)
        pass

    def pose_cb(self, msg):
        self.pos_x =msg.pose.position.x
        self.pos_y =msg.pose.position.y
        self.pos_z =msg.pose.position.z
        if DEBUG:
            print("WAYPOINT UPDATER :: Curr Pos  ","x: ",self.pos_x, "y: ", self.pos_y)
        self.loop();
        
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
        pass
        
    # Find the nearest waypoint ahead based on current position and bearing to next wpt
    def find_next_waypoint(self):
        closestlen = 9999999
        for idx,wpt in enumerate(self.waypoints):
            dist = self.distance_wpt2curr(wpt)
            brg  = self.bearing_wpt2curr(wpt)
            if dist < closestlen and brg > 0:
                self.wpt_ahead     = wpt
                self.wpt_ahead_idx = idx
                closestlen         = dist
        pass
    
    # When a message is recieved from /base_waypoints topic store it
    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints 
        pass    
    
    # Find the distance from a waypoint to current position
    def distance_wpt2curr(self, wpt):
        dist = math.sqrt((self.pos_x-wpt.pose.pose.position.x)**2 + (self.pos_y-wpt.pose.pose.position.y)**2  + (self.pos_z-wpt.pose.pose.position.z)**2)
        return dist
    
    # Find the bearing from current position to waypoint
    def bearing_wpt2curr(self, wpt):
        bearing = math.atan2((wpt.pose.pose.position.y-self.pos_y),(wpt.pose.pose.position.x-self.pos_x))
        return bearing    
        
    # Find the distance between 2 waypoints    
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
