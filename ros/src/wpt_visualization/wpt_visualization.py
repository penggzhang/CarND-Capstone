#!/usr/bin/env python

import rospy
import math
from matplotlib import pyplot
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf
import math

class WPT_Visualization:
    def __init__(self):
        # Initialize node
        rospy.init_node('wpt_visualization')
        self.fig = pyplot.figure(1)
        self.firstpass = True

        # Initialize the /base_waypoints plot
        self.basewpts = pyplot.plot([], [])
        self.routeSub = rospy.Subscriber('/base_waypoints', Lane, self.plotBaseWpts_cb)

        # Initialize the /current_pose
        self.pose = pyplot.plot([], [])
        self.poseSub = rospy.Subscriber('/debug_current_pose', PoseStamped, self.plotCurrPose_cb)

        # Initialize the /final_waypoints plot
        self.finwpts = pyplot.plot([], [])
        self.wptSub  = rospy.Subscriber('/final_waypoints', Lane, self.plotFinalWpts_cb)

        # Set up the on-shutdown functionality
        rospy.on_shutdown(self.kill_viz)
        pass

    def plotBaseWpts_cb(self, msg):
        if self.firstpass:
            self.firstpass = False
            if pyplot.fignum_exists(self.fig.number) and msg.waypoints:
                pyplot.figure(self.fig.number)
                if len(self.basewpts) > 0:
                    self.basewpts.pop(0).remove()
				
                    basewpts_x = []
                    basewpts_y = []
                    for wpt in msg.waypoints:
                        basewpts_x.append(wpt.pose.pose.position.x)
                        basewpts_y.append(wpt.pose.pose.position.y)

                    self.basewpts = pyplot.plot(basewpts_y, basewpts_x, 'b-o', zorder=2)
                    pyplot.draw()
        pass

    def plotCurrPose_cb(self, msg):
        if pyplot.fignum_exists(self.fig.number) and msg.pose.position.x and msg.pose.position.y:
            pyplot.figure(self.fig.number)
            if len(self.pose) > 0:
                self.pose.pop(0).remove()
            self.pose = pyplot.plot(msg.pose.position.y, msg.pose.position.x, 'rs', markersize=5, zorder=5)
            axes = pyplot.gca()
            axes.set_ylim([msg.pose.position.x-10,msg.pose.position.x+10])
            axes.set_xlim([msg.pose.position.y-10,msg.pose.position.y+10])
            pyplot.xlabel('Global Y')
            pyplot.ylabel('Global X')
            pyplot.draw()
        pass
			
    def plotFinalWpts_cb(self, msg):
        if pyplot.fignum_exists(self.fig.number) and msg.waypoints:

            pyplot.figure(self.fig.number)
            if len(self.finwpts) > 0:
                self.finwpts.pop(0).remove()
				
            finalwpts_x = []
            finalwpts_y = []
            for wpt in msg.waypoints:
                finalwpts_x.append(wpt.pose.pose.position.x)
                finalwpts_y.append(wpt.pose.pose.position.y)

            self.finwpts = pyplot.plot(finalwpts_y, finalwpts_x, 'g-o', zorder=2)
            pyplot.draw()
        pass

    def start(self):
        pyplot.figure(self.fig.number)
        pyplot.show()
        pyplot.draw()
        pass

    def kill_viz(self):
        pyplot.close(self.fig)
        rospy.loginfo("Shutting down visualization node")
        pass


def main():   
    # Create the ROS node
    viz = WPT_Visualization()
    
    # Run the visualization
    viz.start()

if __name__ == '__main__':
    main()
