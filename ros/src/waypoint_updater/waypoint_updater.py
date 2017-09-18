#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
import tf
import math
from light_msgs.msg import UpcomingLight
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

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

# TUNABLE PARAMETERS
LOOKAHEAD_WPS = 30       # Number of waypoints published
SPEED_MPH     = 20.      # Forward speed in miles per hour
STOP_LINE     = 29.      # Distance in meters to stop in front of stoplight, corresponds to white stop line
STOP_DIST_ERR = 15.      # Distance to start applying brakes ahead of STOP_LINE

# CONSTANTS
MPH2MPS       = 0.44704  # Conversion miles per hour to meters per second

# DEBUG FLAGS
DEBUG         = False    # True = Print Statements appear in Terminal with Debug info
DEBUG_TOPICS  = False    # Enable debug output topics
SIMULATE_TL   = True     # True = Simulate traffic light positions with /vehicle/traffic_lights, False = use tl_detector /upcoming_light topic
DEBUG_TLDET   = False    # True = Print TL_DETECTOR topic debug information
DEBUG_TLSIM   = False    # True = Print traffic light information from simulator data debug information
DEBUG_TGTSPD  = False    # True = Print debug information for target velocity calculations

class WaypointUpdater(object):
    def __init__(self):
    
        #Initialize node
        rospy.init_node('waypoint_updater')

        # Setup Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.CurrVel_cb,queue_size=1)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        
        # If tl_detector node is active get traffic light data from /upcoming_light topic, otherwise use the topic from the sim /vehicle/traffic_lights
        if SIMULATE_TL:
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.sim_traffic_cb, queue_size=1) 
        else:
            rospy.Subscriber('/upcoming_light',UpcomingLight,self.upcoming_lt_cb,queue_size=1)
            #rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb,queue_size=1)
        
        # Setup Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # Topics to publish only if debugging
        if DEBUG_TOPICS:
            self.debug_currentpos_pub= rospy.Publisher('debug_current_pose', PoseStamped, queue_size=1)

        # Initialize Member Variables
        self.pos_x            = 0.
        self.pos_y            = 0.
        self.pos_z            = 0.
        self.waypoints        = None
        self.wpt_ahead_idx    = 0.       
        self.wpt_ahead        = None
        self.final_wpts       = None
        self.current_orient   = None
        self.light_ahead      = None
        self.target_speed_mps = SPEED_MPH*MPH2MPS
        self.dist2light_m     = 9999.
        rospy.spin()
        
    def loop(self): 
        if self.waypoints != None:
            
            # Find waypoint directly ahead of us and assign it to self.wpt_ahead
            self.find_next_waypoint()
            
            # Initialize the final waypoints ahead of us
            self.final_wpts = Lane()
            
            if False:
                print('WPT Ahead        ',"x: ",self.wpt_ahead.pose.pose.position.x,"y: ",self.wpt_ahead.pose.pose.position.y,"idx: ",self.wpt_ahead_idx)
                print('WPT List Length: ', len(self.waypoints)) 
            
            # Form final waypoint list starting from the waypoint directly ahead
            final_wpt_idx = self.wpt_ahead_idx+LOOKAHEAD_WPS
            if final_wpt_idx < len(self.waypoints): # protect against wrapping around the waypoints array
                self.final_wpts.waypoints = self.waypoints[self.wpt_ahead_idx:final_wpt_idx]
            else:
                self.final_wpts.waypoints = self.waypoints[self.wpt_ahead_idx:len(self.waypoints)]
            
            idx = 0
            # Fill target speeds
            for wpt in self.final_wpts.waypoints:
            
                # Two Checks before we set deceleration profile:
                # 1) Check if we are NOT past the white stop line in the middle of the intersection => dont stop in middle
                # 2) Check if we close enough to the stop line to start braking
                # If either are false then just continue at normal speed
                dist2stopline   = self.dist2light_m - STOP_LINE
                in_intersection = dist2stopline < -1. # arbitrary tuned number
                start_braking   = math.fabs(dist2stopline) < STOP_DIST_ERR 
                if DEBUG_TGTSPD:
                    print('dist2stopline',dist2stopline)
                    print('---------------------------')
                if start_braking and not in_intersection:
                    # For each waypoint compute in order:
                        # Distance to the stoplight from waypoint
                        # Distance from waypoint to stopline in front of stoplight
                        # Desired deceleration
                        # Final velocity command for waypoint
                    # Find the distance from the wpt to the stoplight
                    dis_wpt2light = math.sqrt((wpt.pose.pose.position.x - self.light_ahead.pose.pose.position.x)**2 + (wpt.pose.pose.position.y-self.light_ahead.pose.pose.position.y)**2)
                    # Find the desired final stop position of the car
                    pos_f = dis_wpt2light - STOP_LINE #stop at the white line in front of the light
                    vel_i = self.current_velocity.linear.x
                    # If velocity is small then dont decelerate
                    if vel_i < 0.1:
                        vel_i = 0.
                        decel_mpss = 0
                    else:
                        decel_mpss = -1.*(vel_i * vel_i / pos_f)/2. # Find the required deceleration
                   
                    # Check if we are already past the white line
                    if pos_f < 0.0:
                        pos_f = 0.0
                        self.target_speed_mps = 0.0
                    else: 
                        vel_sq = vel_i * vel_i + 2.*decel_mpss*pos_f
                        if vel_sq < 0.:
                            vel_sq = 0.
                        self.target_speed_mps = math.sqrt(vel_sq)
                        
                    if DEBUG_TGTSPD and math.fabs(self.dist2light_m - STOP_LINE) < STOP_DIST_ERR:
                        print('TGT SPDS :: Final WPT IDX     ', idx) 
                        print('TGT SPDS :: Target Speed      ', self.target_speed_mps) 
                        print('TGT SPDS :: Pos to Stop Ahead ', pos_f) 
                        print('TGT SPDS :: Dist car2light     ', self.dist2light_m)
                        print('TGT SPDS :: Dist wpt2light     ', dis_wpt2light)
                        idx = idx + 1  
                    
                else:
                    # Car goes at normal speed
                    self.target_speed_mps = SPEED_MPH*MPH2MPS
                
                # Set speeds in waypoint list
                wpt.twist.twist.linear.x = self.target_speed_mps
                    
            #Publish final waypoints
            self.final_wpts.header.stamp    = rospy.Time.now()
            self.final_waypoints_pub.publish(self.final_wpts)
            
            #Topics to publish for debugging
            if DEBUG_TOPICS:
                self.debug_currpos                  = PoseStamped()
                self.debug_currpos.header.stamp     = rospy.Time.now()
                self.debug_currpos.pose.position.x  = self.pos_x
                self.debug_currpos.pose.position.y  = self.pos_y
                self.debug_currpos.pose.position.z  = self.pos_z
                self.debug_currpos.pose.orientation = self.current_orient
                self.debug_currentpos_pub.publish(self.debug_currpos)
        pass

    def pose_cb(self, msg):
        self.pos_x          = msg.pose.position.x
        self.pos_y          = msg.pose.position.y
        self.pos_z          = msg.pose.position.z
        self.current_orient = msg.pose.orientation
        self.loop();
        
        if False:
            print("WAYPOINT UPDATER :: Curr Pos  ","x: ",self.pos_x, "y: ", self.pos_y)
        
        pass
        
    def upcoming_lt_cb(self,msg):
        self.light_ahead     = msg.pose
        self.light_ahead_idx = msg.waypoint
        self.dist2light_m    = math.sqrt((self.pos_x - self.light_ahead.pose.pose.position.x)**2 + (self.pos_y-self.light_ahead.pose.pose.position.y)**2)
        if DEBUG_TLDET:
            print("From TL_DETECTOR :: Light Ahead IDX  ",self.light_ahead_idx)
            print("From TL_DETECTOR :: Light Ahead DIST ",self.dist2light_m)
        pass
    
    def traffic_cb(self,msg):
        # We are already getting this from the /upcoming_light topic
        # self.light_ahead_idx = msg.data
        pass

    def sim_traffic_cb(self, msg):
        #Find closest light ahead of us
        closestlen = 9999999
        if self.current_orient != None:
            for idx,light in enumerate(msg.lights):
                dist = self.distance_wpt2curr(light)
                brg  = self.bearing_wpt2curr(light)
                #Find closest light, in front of us, that is either red or yellow
                if dist < closestlen and brg > 0.0 and (light.state == 0 or light.state == 1):
                    self.light_ahead     = light
                    self.light_ahead_idx = idx
                    closestlen = dist
            self.dist2light_m    = closestlen
            
            if DEBUG_TLSIM and self.light_ahead != None:
                print("SIM TL :: Light Ahead IDX          ", self.light_ahead_idx)
                print("SIM TL :: Light Ahead DIST         ", self.dist2light_m)
                
        #Find nearest waypoint to closest light
        if self.waypoints != None and self.light_ahead != None:
            closestlen = 9999999
            for idx,wpt in enumerate(self.waypoints):
                dist = self.distance_2wpts(self.light_ahead,wpt)
                #Find waypoint that is closest to the nearest red light we just found
                if dist < closestlen:
                    self.wpt_light_ahead     = wpt
                    self.wpt_light_ahead_idx = idx
                    closestlen = dist
        
            if DEBUG_TLSIM:
                print("SIM TL :: Nearest WPT to Light IDX ", self.wpt_light_ahead_idx)
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
            brg= self.bearing_wpt2curr(wpt)
            #if dist < closestlen and (math.pi/2-brg) < math.pi/4: #check if waypoint is directly ahead within some window
            if dist < closestlen and brg > 0.0:
                self.wpt_ahead     = wpt
                self.wpt_ahead_idx = idx
                closestlen         = dist
                stored_brg         = brg
        pass
    
    # When a message is recieved from /base_waypoints topic store it
    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        #only need to set /base_waypoints once, unsubscribe to improve performance 
        self.base_waypoints_sub.unregister()
        pass    
    
    # Find the distance from a waypoint to the car's current position
    def distance_wpt2curr(self, wpt):
        dist = math.sqrt((self.pos_x-wpt.pose.pose.position.x)**2 + (self.pos_y-wpt.pose.pose.position.y)**2  + (self.pos_z-wpt.pose.pose.position.z)**2)
        return dist
        
    # Find the distance between 2 specific waypoints   
    def distance_2wpts(self, wpt1, wpt2):
        dist = math.sqrt((wpt2.pose.pose.position.x-wpt1.pose.pose.position.x)**2 + (wpt2.pose.pose.position.y-wpt1.pose.pose.position.y)**2  + (wpt2.pose.pose.position.z-wpt1.pose.pose.position.z)**2)
        return dist
    
    # Find the bearing from current position to waypoint, need to rotate to car body frame
    def bearing_wpt2curr(self, wpt):
        global_car_x = self.pos_x
        global_car_y = self.pos_y
        
        roll,pitch,yaw = tf.transformations.euler_from_quaternion([self.current_orient.x,self.current_orient.y,self.current_orient.z,self.current_orient.w])
        yaw = -1.*yaw 

        self.shiftx = wpt.pose.pose.position.x - global_car_x
        self.shifty = wpt.pose.pose.position.y - global_car_y

        self.del_x =  (self.shiftx)*math.cos(yaw) - (self.shifty)*math.sin(yaw)
        self.del_y =  (self.shiftx)*math.sin(yaw) + (self.shifty)*math.cos(yaw)  
  
        bearing = math.atan2(self.del_x,self.del_y)
        return bearing
        
    # Find the distance between 2 waypoints in a series of waypoints   
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
        
    # When a message is recieved from /current_velocity topic store it
    def CurrVel_cb(self, msg):
        self.current_velocity = msg.twist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
