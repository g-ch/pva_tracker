# Introduction

This package contains a tracker that receives position, velocity, and acceleration commands and turns the commands to attitude and thrust setpoints.
These setpoints will be sent to Mavros to control a PX4 drone.
So Mavros should be started before using this tracker.

__NOTE:__
Before you use the tracker for a real drone, the parameter `hover_thrust_factor_min, hover_thrust_factor_max, "flight_time_minute` and `stand_by_time_minute` in `cfg/pid.cfg` should be set to a right value.

+ "flight_time_minute" and "stand_by_time_minute" are the maximum flight time and maximum stand-by time (when the drone is disarmed but the onboard computer is running.)


+ "hover_thrust_factor_min" and "hover_thrust_factor_max" correspond to the hover thrust factor when the battery is fully charged and almost empty.
  Our hover thrust is `px4's hover thrust / 9.8`.



# Compile
Clone to your ROS workspace and run
```
catkin_make
```
or
```
catkin build
```

# Usage

## Nodes Introduction
Two nodes for the tracker:

+ tracker: receive position, velocity, and acceleration commands and publish attitude and thrust commands. The topic name is `/pva_setpoint`. The message type is `trajectory_msgs::JointTrajectoryPoint`, where the fourth element in the position vector is yaw setpoint.


+ tracker_sim_auto_arm_takeoff: __Use this node only in simulation.__ This node will arm and take off the drone automatically and wait for the same commands as the tracker node uses.


Two nodes to generate hover commands or simple trajectories. These two nodes only work when one of the above nodes has been started.

+ hover_test: record the position when the drone is set to `OFFBOARD` mode and hover.


+ control_test: generates trajectories with four stages when the drone is armed and set to offboard mode:
    1. Takeoff
    2. Get to a point (R, -R) by minimum jerk planner
    3. Accelerate to v and get to point (R, 0)
    4. Continuously fly as a circle with radius R, Speed v.


## Run
1.Run the tracker by:
```
rosrun pva_tracker tracker
```

or (only in simulation)
```
rosrun pva_tracker tracker_sim_auto_arm_takeoff
```
Then you can run your planner to generate pva commands.


2.If you want to test basic hovering performance in OFFBOARD mode, launch by
```
roslaunch pva_tracker hover_test.launch
```
Then fly your drone to a desired position and switch to OFFBOARD mode. If the parameters are good, the drone will hover at the position.
The yaw direction depends on a parameter you set in the launch file.
__The default yaw is zero.__


3.If you want to test auto takeoff and draw a circle with the control_test node, launch by
```
roslaunch pva_tracker take_off_and_draw_circle.launch
```
__Parameters like circle radius and flight speed can be set in the launch file.__

##  Tune PID parameters:
The default parameters are stored in `cfg/pid.cfg`.
You can tune the parameter using a rqt tool:
```
rosrun rqt_reconfigure rqt_reconfigure 
```
When you change any parameter with the rqt_reconfigure, the changing takes effect immediately.
Therefore, __Never change any parameter when the drone is flying.__

Changes through `rqt_reconfigure` will not be saved when ROS is shut down.
Therefore, save the parameters as a yaml file so that you can reload the parameters with rqt_reconfigure next time.
When you have finished parameter tuning, you can change the default parameters in `cfg/pid.cfg`. Changes to `cfg/pid.cfg` require recompilation and restarting ROS to take effect.






