#!/usr/bin/env python

import rospy
import math
from matplotlib import pyplot
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from matplotlib import rc

class control_visualization:
    def __init__(self):
        # Initialize node
        rospy.init_node('control_visualization')
        self.fig          = pyplot.figure(2)
        self.ax1          = self.fig.add_subplot(311)
        self.ax2          = self.fig.add_subplot(312)
        self.ax3          = self.fig.add_subplot(313)
        self.firstpass    = True
        
        rc('axes', linewidth=2)
        rc('font', weight='bold')

        self.steer_pts    = pyplot.plot([], [])
        self.steer_sub    = rospy.Subscriber('/vehicle/steering_cmd',SteeringCmd, self.plot_strcmd_cb)
        self.thr_pts      = pyplot.plot([], [])
        self.throttle_sub = rospy.Subscriber('/vehicle/throttle_cmd',ThrottleCmd, self.plot_thrcmd_cb)
        self.brk_pts      = pyplot.plot([], [])
        self.brake_sub    = rospy.Subscriber('/vehicle/brake_cmd',   BrakeCmd,    self.plot_brkcmd_cb)
        
        self.brk_count    = 0
        self.str_count    = 0
        self.thr_count    = 0

        # Set up the on-shutdown functionality
        rospy.on_shutdown(self.kill_viz)
        pass

    def plot_thrcmd_cb(self, msg):
        if pyplot.fignum_exists(self.fig.number) and msg.pedal_cmd:
            pyplot.figure(self.fig.number)
            self.ax1.plot(self.thr_count,msg.pedal_cmd, 'ro')
            self.ax1.set_ylabel('Thr Cmd (%)',fontsize = 10)
            pyplot.draw()
            
            self.thr_count = self.thr_count + 1
        pass
    
    def plot_strcmd_cb(self, msg):
        if pyplot.fignum_exists(self.fig.number) and msg.steering_wheel_angle_cmd:
            pyplot.figure(self.fig.number)
            self.ax2.plot(self.str_count,msg.steering_wheel_angle_cmd, 'ro')
            self.ax2.set_ylabel('Str Cmd (rad)',fontsize = 10)
            pyplot.draw()
            
            self.str_count = self.str_count + 1
        pass
    
    def plot_brkcmd_cb(self, msg):
        if pyplot.fignum_exists(self.fig.number) and msg.pedal_cmd:
            pyplot.figure(self.fig.number)
            self.ax3.plot(self.brk_count,msg.pedal_cmd, 'ro')
            self.ax3.set_ylabel('Brk Cmd (Nm)}',fontsize = 10)
            pyplot.draw()
            
            self.brk_count = self.brk_count + 1
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
    viz = control_visualization()
    
    # Run the visualization
    viz.start()

if __name__ == '__main__':
    main()
