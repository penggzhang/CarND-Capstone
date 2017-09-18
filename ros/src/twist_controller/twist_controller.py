import pid
import lowpass
import math
import yaw_controller as yc
import rospy

#GAS_DENSITY = 2.858   # TODO: Figure out why we need this - fuel in tank sloshing?
#ONE_MPH     = 0.44704 # MPH to M/S conversion
DEBUG_STEER = False

class Controller(object):

    def __init__(self, *args, **kwargs):
        # Timesteps
        self.dt       = 0.1 # 10Hz
        self.prevtime = rospy.get_time() 
        # TODO: Pull PID Gains out to Launch Files
        # TODO: Tune Coefficients
        # Initialize Gains on PID Controllers
        #self.PIDCont_Thr = pid.PID(kp=1.5,ki=0.02,kd=0.1,mn= 0.0,mx=1.0) #starting point for tuning
        self.PIDCont_Thr = pid.PID(kp=1.0,ki=0.04,kd=0.1,mn= 0.0,mx=1.0)
        self.PIDCont_Brk = pid.PID(kp=200.0,ki=0.00,kd=0.0,mn= 0.0,mx=10000.0)
        #self.PIDCont_Str = pid.PID(kp=0.7,ki=0.001,kd=0.4,mn=-8.0,mx=8.0) #starting point for tuning
        #self.PIDCont_Str = pid.PID(kp=0.14,ki=0.01,kd=0.2,mn=-8.0,mx=8.0) #starting point for tuning (2)
        self.PIDCont_Str = pid.PID(kp=0.14,ki=0.0,kd=2.0,mn=-8.0,mx=8.0)
        self.YawCont_Str = yc.YawController(wheel_base=2.8498, steer_ratio=14.8, min_speed=10.0, max_lat_accel=3.0, max_steer_angle=8.0)
        # Initialize Low Pass Filters
        self.LPFilt_Thr  = lowpass.LowPassFilter(tau=0.0,ts=self.dt)
        self.LPFilt_Brk  = lowpass.LowPassFilter(tau=0.0,ts=self.dt)
        self.LPFilt_Str  = lowpass.LowPassFilter(tau=0.5,ts=self.dt)
        self.first = True # first pass flag
        pass

    def control(self, proposed_linear_velocity,proposed_angular_velocity,current_linear_velocity,cross_track_error,heading_error,dbw_status):
        if dbw_status:
            # Compute dt
            self.dt       = rospy.get_time() - self.prevtime
            self.prevtime = rospy.get_time()
            if self.dt < 1e-3:
                self.dt = 0.1
        
            throttle = 0.0 
            brake    = 0.0
            steer    = 0.0 # radians -8.2 to 8.2, 14.8 steering gear ratio implies ~31 degrees of play

            # Clip the proposed angular velocity to reasonable values
            proposed_angular_velocity_clip = proposed_angular_velocity
            if proposed_angular_velocity_clip > 1.0:
                proposed_angular_velocity_clip = 1.0
            if proposed_angular_velocity_clip < -1.0:
                proposed_angular_velocity_clip = -1.0

            # Clip the proposed linear velocity to reasonable values
            proposed_linear_velocity_clip = proposed_linear_velocity
            if proposed_linear_velocity_clip < 0.0:
                proposed_linear_velocity_clip = -proposed_linear_velocity
           
            # Compute velocity error
            vel_err  = proposed_linear_velocity_clip-current_linear_velocity
            if False:
                print('vel_err',vel_err)
            if vel_err <= 0.:
                brake = self.PIDCont_Brk.step(-1.*vel_err,self.dt)
                #brake = self.LPFilt_Brk.filt(brake)
                # don't control throttle if braking
                self.PIDCont_Thr.reset()
                if False:
                    print('Brake',brake)
            else:
                throttle = self.PIDCont_Thr.step(vel_err,self.dt)
                #throttle = self.LPFilt_Thr.filt(throttle)
                # don't control brake if throttling
                self.PIDCont_Brk.reset()
                if False:
                    print('Throttle',throttle)

            # With yaw controller, this probably needs smaller gains, possibly mostly integral action
            cte_steer = self.PIDCont_Str.step(cross_track_error,self.dt)

            pred_steer = self.YawCont_Str.get_steering(proposed_linear_velocity_clip, proposed_angular_velocity_clip, current_linear_velocity)
            
            #print('    PID steer command :',cte_steer)
            #print('Raw Yaw steer command :',pred_steer)
            
            pred_steer_flt = self.LPFilt_Str.filt(pred_steer) # filter the "feedforward term"
            steer = pred_steer_flt + cte_steer

            if DEBUG_STEER:
                # Create a Matlab format log file to determine issue with proposed_angular velocity
                if self.first:
                    try:
                        f = open('datalog.m', 'r+')
                    except:
                        f = open('datalog.m', 'w')
                    self.first = False
                    f.write('data = [...\n')
                else:
                    try:
                        f = open('datalog.m', 'a+')
                    except:
                        f = open('datalog.m', 'w')
                        self.first = False
                        f.write('data = [...\n')
                        
                f.write(str(proposed_angular_velocity))         # Matlab index 1
                f.write(', ')
                f.write(str(proposed_angular_velocity_clip))    # Matlab index 2
                f.write(', ')
                f.write(str(proposed_linear_velocity))          # Matlab index 3
                f.write(', ')
                f.write(str(proposed_linear_velocity_clip))     # Matlab index 4
                f.write(', ')
                f.write(str(brake))                             # Matlab index 5
                f.write(', ')
                f.write(str(throttle))                          # Matlab index 6
                f.write(', ')
                f.write(str(vel_err))                           # Matlab index 7
                f.write(', ')
                f.write(str(cte_steer))                         # Matlab index 8
                f.write(', ')
                f.write(str(cross_track_error))                 # Matlab index 9
                f.write(', ')
                f.write(str(pred_steer))                        # Matlab index 10
                f.write(', ')
                f.write(str(pred_steer_flt))                    # Matlab index 11
                f.write(', ')
                f.write(str(steer))                             # Matlab index 12
                f.write(';...\n')
                f.close()
            # end if DEBUG_STEER:
            
            #print('Flt Yaw steer command :',pred_steer)
            #print('  Total steer command :',steer)
        else:
            #DBW is not enabled so manual steering, reset integrators and low pass filters
            self.ResetLPFs()
            self.ResetPIDs()
            # Do not supply values for throttle, brake or steering
            throttle = 0.0
            brake    = 0.0
            steer    = 0.0
            
        return throttle, brake, steer
            
    def ResetPIDs(self):
        self.PIDCont_Thr.reset()
        self.PIDCont_Brk.reset()
        self.PIDCont_Str.reset()
        pass 
                
    def ResetLPFs(self):
        self.LPFilt_Thr  = lowpass.LowPassFilter(tau=0.0,ts=self.dt)
        self.LPFilt_Brk  = lowpass.LowPassFilter(tau=0.0,ts=self.dt)
        self.LPFilt_Str  = lowpass.LowPassFilter(tau=0.5,ts=self.dt)
        pass
