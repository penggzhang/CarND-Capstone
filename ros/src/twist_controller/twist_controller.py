import pid
import lowpass
import yaw_controller as yc

#GAS_DENSITY = 2.858   # TODO: Figure out why we need this - fuel in tank sloshing?
#ONE_MPH     = 0.44704 # MPH to M/S conversion

class Controller(object):

    def __init__(self, *args, **kwargs):
        # TODO: Pull PID Gains out to Launch Files
        # TODO: Tune Coefficients
        # Initialize Gains on PID Controllers
        self.PIDCont_Thr = pid.PID(kp=1.0,ki=0.04,kd=0.1,mn= 0.0,mx=1.0)
        self.PIDCont_Brk = pid.PID(kp=0.0,ki=0.00,kd=0.0,mn= 0.0,mx=1.0)
        self.PIDCont_Str = pid.PID(kp=0.01,ki=0.001,kd=0.1,mn=-8.0,mx=8.0)
        self.YawCont_Str = yc.YawController(wheel_base=2.8498, steer_ratio=14.8, min_speed=10.0, max_lat_accel=3.0, max_steer_angle=8.0)
        # Initialize Low Pass Filters
        self.LPFilt_Thr  = lowpass.LowPassFilter(tau=0.0,ts=1.00)
        self.LPFilt_Brk  = lowpass.LowPassFilter(tau=0.0,ts=1.00)
        self.LPFilt_Str  = lowpass.LowPassFilter(tau=0.0,ts=1.00)
        # TODO: Calculate timestep in real time
        self.dt = 0.02 # 50Hz
        pass

    def control(self, proposed_linear_velocity,proposed_angular_velocity,current_linear_velocity,cross_track_error,dbw_status):
        if dbw_status:
            throttle = 0.0
            brake    = 0.0
            steer    = 0.0
            
            vel_err  = proposed_linear_velocity-current_linear_velocity
            throttle = self.PIDCont_Thr.step(vel_err,self.dt)
            #throttle = self.LPFilt_Thr.filt(throttle)
            
            steer = self.PIDCont_Str.step(cross_track_error,self.dt)
            # TODO: Figure out how to use this properly
            #steer = self.YawCont_Str.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
            #steer = self.LPFilt_Str.filt(steer)
            
            # TODO: Figure out how to use the brakes
            #brake = self.PIDCont_Brk.step(vel_err,self.dt)
            #brake = 0 #self.LPFilt_Brk.filt(brake)
            
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
        self.LPFilt_Thr  = lowpass.LowPassFilter(tau=0.0,ts=1.0)
        self.LPFilt_Brk  = lowpass.LowPassFilter(tau=0.0,ts=1.0)
        self.LPFilt_Str  = lowpass.LowPassFilter(tau=0.0,ts=1.0)
        pass
