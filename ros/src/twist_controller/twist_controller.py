import pid
import lowpass
import math
import yaw_controller as yc
import rospy

#GAS_DENSITY = 2.858   # TODO: Figure out why we need this - fuel in tank sloshing?
ONE_MPH     = 0.44704 # MPH to M/S conversion
DEBUG_STEER = False
STEER_CTE = True # True uses CTE error for PID, False uses angular velocity error

class Controller(object):

    def __init__(self, *args, **kwargs):
        # Timesteps
        self.dt       = 0.1 # 10Hz
        self.prevtime = rospy.get_time() 
        # TODO: Pull PID Gains out to Launch Files
        # TODO: Tune Coefficients
        # Initialize Gains on PID Controllers
        self.PIDCont_Thr = pid.PID(kp=1.0,ki=0.04,kd=0.1,mn= 0.0,mx=1.0)
        self.PIDCont_Brk = pid.PID(kp=200.0,ki=0.00,kd=0.0,mn= 0.0,mx=10000.0)
        if STEER_CTE:
            # cross track error based steering
            self.PIDCont_Str = pid.PID(kp=0.25,ki=0.075,kd=1.5,mn=-0.5,mx=0.5)
        else:
            # angular velocity error based steering
            self.PIDCont_Str = pid.PID(kp=4.0,ki=0.5,kd=0.05,mn=-0.5,mx=0.5)
            
        self.YawCont_Str = yc.YawController(wheel_base=2.8498, steer_ratio=14.8, min_speed=10.0, max_lat_accel=3.0, max_steer_angle=8.0)
        
        # Initialize Low Pass Filters
        self.LPFilt_Thr  = lowpass.LowPassFilter(tau=0.0,ts=self.dt)
        self.LPFilt_Brk  = lowpass.LowPassFilter(tau=0.0,ts=self.dt)
        if STEER_CTE:
            # cross track error based steering
            self.LPFilt_Str  = lowpass.LowPassFilter(tau=0.7,ts=self.dt)
        else:
            # angular velocity error based steering
            self.LPFilt_Str  = lowpass.LowPassFilter(tau=0.5,ts=self.dt)

        self.first = True # first pass flag
        pass

    def control(self, proposed_linear_velocity,proposed_angular_velocity,current_linear_velocity,current_angular_velocity,cross_track_error,heading_error,dbw_status,):
        if dbw_status:
            # Compute dt
            self.dt       = rospy.get_time() - self.prevtime
            self.prevtime = rospy.get_time()
            if self.dt < 1e-3:
                self.dt = 0.1
        
            throttle = 0.0 
            brake    = 0.0
            steer    = 0.0 # radians -8.2 to 8.2, 14.8 steering gear ratio implies ~31 degrees of play

            if not STEER_CTE:           
                # Compute angular velocity error
                ang_err = proposed_angular_velocity - current_angular_velocity

                # deadband
                if ang_err >= 0.005:
                    ang_err -= 0.005
                elif ang_err <= -0.005:
                    ang_err += 0.005
                else:
                    ang_err = 0.0

                # upper limit
                if ang_err >= 0.25:
                    ang_err = 0.25
                elif ang_err <= -0.25:
                    ang_err += 0.25

                ang_err = self.LPFilt_Str.filt(ang_err)

            # Clip the proposed angular velocity to reasonable values
            proposed_angular_velocity_clip = proposed_angular_velocity
            if proposed_angular_velocity_clip > 1.0:
                proposed_angular_velocity_clip = 1.0
            if proposed_angular_velocity_clip < -1.0:
                proposed_angular_velocity_clip = -1.0

            # Clip the proposed linear velocity to reasonable values
            # Anomalies seen in data with this signal
            proposed_linear_velocity_clip = proposed_linear_velocity
            if proposed_linear_velocity_clip < 0.0:
                proposed_linear_velocity_clip = -proposed_linear_velocity
           
            # Compute linear velocity error
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

            # Feedforward steering based on waypoints
            # and desired angular velocity
            pred_steer = self.YawCont_Str.get_steering(proposed_linear_velocity_clip, proposed_angular_velocity_clip, current_linear_velocity)

            # inhibit integrator wind-up if car is stationary
            if current_linear_velocity > ONE_MPH:
                if STEER_CTE:
                    pid_steer = self.PIDCont_Str.step(cross_track_error,self.dt)
                else:
                    pid_steer = self.PIDCont_Str.step(ang_err,self.dt)
                
            else:
                self.PIDCont_Str.reset()
                pid_steer = 0.0

            # Feedback correction
            steer = pred_steer + pid_steer

            if DEBUG_STEER:
                if STEER_CTE:
                    print('CTE:      ', cross_track_error)
                else:
                    print('Angular velocity error ',ang_err)
                print('FF steer: ', pred_steer)
                print('FB steer: ', pid_steer)
                print('P term:   ', self.PIDCont_Str.pterm)
                print('I term:   ', self.PIDCont_Str.iterm)
                print('D term:   ', self.PIDCont_Str.dterm)
            
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
        self.LPFilt_CTE  = lowpass.LowPassFilter(tau=0.2,ts=self.dt)
        pass
