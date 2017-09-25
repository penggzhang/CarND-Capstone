# TEAM AURORA: SDCND CAPSTONE PROJECT

### Team Leader: Siva Kondubhatla (sivakirank@gmail.com)

### Team Members: 
* Andrew Stewart (andrew.stewart2@honeywell.com) 
* Jim Reynolds (jreyn121@ford.com) 
* Peng Zhang (penggzhang@126.com)
* Francisco Jose Rey Gozalo (fcojreyg@gmail.com)

This is the Team Aurora project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Installation 

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop). 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository

Project Code:
```bash
git clone https://github.com/DFStewart/CarND-Capstone.git
```

Udacity Original Project Code:
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

## Waypoint Update
The geographical `x,y,z` position of the path waypoints are provided.  The waypoint updater must calculate the desired velocity (m/sec) at each of the waypoints.  

For the case where there is no red light within the stop horizon `STOP_DIST_ERR` the desired velocity determination is trivially set to a constant value `SPEED_MPH*MPH2MPS`.

For the case where there is a stop light ahead, the determination of the desired velocity at the upcoming waypoints is more involved.  The key concept is the planning determines the constant deceleration necessary to stop at the stop line, then back calculates the velocity of each waypoint leading up to the stop line.

The constant deceleration is calculated as shown below:
```Python
decel_mpss = -1.*(vel_i * vel_i / pos_stopline)/2
```

For each waypoint between the vehicle's current location and the stopline, `decel_mpss` is then used to calculate the desired velocity as~
```Python
vel_sq = vel_i * vel_i + 2.*decel_mpss*pos_stopline
if vel_sq < 0.0:
    vel_sq = 0.0
self.target_speed_mps = math.sqrt(vel_sq)
```

Detailed derivations of these calculation are available here %%%%%.

## Control Systems
There are three closed-loop controllers present for the vehicle motion control:
1. Throttle (accelerator pedal)
2. Braking
3. Steering

Each of these uses a PID controller.  The throttle and braking controllers are mutually exclusive; when one of the controllers is active the other is held in reset, and vice versa.

### Throttle
The throttle controller is strictly a feedback controller.  There is no feedforward throttle calculation based on speed sepoint.  The velocity error is the current desired linear velocity (m/sec) minus the current linear velocity (m/sec).  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

The throttle controller is active when `vel_err > 0.0`.  Otherwise, the throttle controller is reset, and the brake controller is utilized.

### Brake
The brake controller is strictly a feedback controller.  There is no feedforward brake calculation based on speed sepoint.  The velocity error is the current desired linear velocity (m/sec) minus the current linear velocity (m/sec).  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

The brake controller is active when `vel_err <= 0.0`.  Otherwise, the brake controller is reset, and the throttle controller is utilized.

### Steering
The steering control system has 3 main components:
1. Angular velocity setpoint determination
2. Feedforward steering angle command
3. Feedback steering angle command

Setpoint determination is done within the `waypoint_follower` module, specifically in `pure_pursuit_core.cpp`.  The function fits a curve to the desired waypoints, then uses the curvature (rad/m) and desired linear velocity (m/sec) to calculate the corresponding angular velocity (rad/sec).

The feedforward steering angle command is calculated in the `twist_controller` module, in `twist_controller.py` and `yaw_controller.py`. transforms the desired angular velocity (rad/sec), desired linear velocity (m/sec) and current linear velocity (m/sec) into a steering wheel angle (rad).  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

The feedback steering angle command is calculated in `twist_controller.py`.  Two different methods were evaluated for this controller:
1. Cross track error (CTE) based PID control
2. Angular velocity error based PID control

#### Cross Track Error Based PID Control
The error is calculated as the normal distance between the vehicle and line-of-best-fit of the waypoints, with a 3rd order polynomial.  PID control is applied to this error term.  CTE is calculated in `dbw_node.py`.

```Python
def Compute_CTE(self):
    # Similar to the MPC project we need to convert from global coordinates to local car body coordinates
    # First translate and then rotate
        
    global_car_x = self.current_pose.x
    global_car_y = self.current_pose.y
        
    roll,pitch,yaw = tf.transformations.euler_from_quaternion([self.current_orient.x,self.current_orient.y,self.current_orient.z,self.current_orient.w])
    yaw = -1.*yaw
        
    x_car_body = []
    y_car_body = []
        
    for idx,wpt in enumerate(self.finalwpts):
        del_x = wpt.pose.pose.position.x - global_car_x
        del_y = wpt.pose.pose.position.y - global_car_y
            
        temp1 =  del_x*math.cos(yaw) - del_y*math.sin(yaw)
        temp2 =  del_x*math.sin(yaw) + del_y*math.cos(yaw)
            
        x_car_body.append(temp1)
        y_car_body.append(temp2)
        
    # As with MPC project, fit a 3rd order polynomial that fits most roads    
    coeff_3rd = np.polyfit(x_car_body,y_car_body,3)       
    CTE       = np.polyval(coeff_3rd,0.0)
        
    #Limit CTE to +/-5 which is empirically the limits of the lanes 0 and 2
    if CTE > 5.0:
        CTE = 5.0
    if CTE < -5.0:
        CTE = -5.0       
        
    return CTE
```

#### Angular Velocity Error Based PID Control
The error is calculated as the difference between the measured angular velocity and the desired angular velocity.  PID control is applied to this error term.  A deadzone is applied to the angular velocity error in order to minimize steering corrections (integral windup, oscillation) when very near the setpoint.  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

```Python
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
```

#### Final Steering Control Design
The CTE based control was chosen in the end as it performed more robustly to momentary spikes in error signal (these may be artifacts of the simulator, or discontinuities in the waypoint calculations).
