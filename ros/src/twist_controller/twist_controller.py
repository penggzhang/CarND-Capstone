import pid
import lowpass

# TODO: Figure out why we need these?
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Pull PID Gains out to Launch Files
        # TODO: Tune Coefficients
        # Initialize Gains on PID Controllers
        self.PIDCont_Thr = pid.PID(kp=1.0,ki=0.0,kd=0.0,mn=0.0,mx=1.0)
        self.PIDCont_Brk = pid.PID(kp=0.0,ki=0.0,kd=0.0,mn=0.0,mx=1.0)
        self.PIDCont_Str = pid.PID(kp=0.0,ki=0.0,kd=0.0,mn=-1.0,mx=1.0)
        # Initialize Low Pass Filters
        self.LPFilt_Thr  = lowpass.LowPassFilter(tau=0.0,ts=1.00)
        self.LPFilt_Brk  = lowpass.LowPassFilter(tau=0.0,ts=1.00)
        self.LPFilt_Str  = lowpass.LowPassFilter(tau=0.0,ts=1.00)
        # TODO: Calculate timestep in real time
        self.dt = 0.02
        pass

    def control(self, proposed_linear_velocity,proposed_angular_velocity,current_linear_velocity,dbw_status):
        if dbw_status:
            throttle = 0
            brake    = 0
            steer    = 0
            
            vel_err  = proposed_linear_velocity-current_linear_velocity
            #print('TWIST_CONTROLLER:: VEL_ERROR: ',vel_err)
            throttle = self.PIDCont_Thr.step(vel_err,self.dt)
            #throttle = self.LPFilt_Thr.filt(throttle)
            
            cross_track_err  = proposed_angular_velocity
            steer = self.PIDCont_Str.step(vel_err,self.dt)
            #steer = self.LPFilt_Str.filt(steer)
            
            # TODO: Figure out how to use the brakes
            brake = self.PIDCont_Brk.step(vel_err,self.dt)
            brake = 0 #self.LPFilt_Brk.filt(brake)
            
            return throttle, brake, steer
        else:
            #DBW is not enabled so manual steering, reset integrators and low pass filters
            self.ResetLPFs()
            self.ResetPIDs()
            return 0., 0., 0.
            
    def ResetPIDs(self):
        self.PIDCont_Thr.reset()
        self.PIDCont_Brk.reset()
        self.PIDCont_Str.reset()
        pass 
                
    def ResetLPFs(self):
        self.LPFilt_Thr  = lowpass.LowPassFilter(tau=1.0,ts=1.0)
        self.LPFilt_Brk  = lowpass.LowPassFilter(tau=1.0,ts=1.0)
        self.LPFilt_Str  = lowpass.LowPassFilter(tau=1.0,ts=1.0)
        pass
