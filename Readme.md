This package receives position, velocity, and acceleration commands and turn the commands to attitude and thrust setpoints. These setpoints will be sent to Mavros to control a PX4 drone. 

# Simulation

See the below website to learn more about simulation:
https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html

## Start simulation world
Open the folder of PX4 code and run:

```
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

make px4_sitl_default gazebo
```

## Start mavros:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

## Start tracker by:
```
rosrun pva_tracker tracker 
```

or run "tracker_sim_auto_arm_takeoff" to arm and hover before waiting for position, velocity, and acceleration commands.

```
rosrun pva_tracker tracker_sim_auto_arm_takeoff
```

##  Test flight
run the following to accelerate and fly as a circle (tracker must be started first)
```
rosrun pva_tracker sim_control_test
```

run the following to hover
```
rosrun pva_tracker sim_hover_test
```

##  Tune PID parameters:
```
rosrun rqt_reconfigure rqt_reconfigure 
```

# Real world test
In real world tests, do not use auto arm with code. Always arm and change mode with a remote controller.

## Run pva_tracker in a laptop
In this case, you connect your laptop with your drone via wifi/telemetry and Mavros. The estimated position and velocity are given by optitrack system.

Run the following to start tracker and takeoff the drone when the mode is turned to "offboard".
```
rosrun pva_tracker tracker_auto_takeoff_optitrack
```

Now turn the flight mode to "OFFBOARD". The drone will take off, hover and wait for pva commands. 


## Run pva_tracker in an onboard computer
In this case, you connect your onboard computer with your flight controller via usb_to_ttl module and Mavros. The estimated position and velocity are given by tracking camera, SLAM algorithms, or optitrack system. 

Run the following to start tracker:
```
rosrun pva_tracker tracker
```

Run your own planning algorithms and publish position, velocity, and acceleration commands with message type "trajectory_msgs/JointTrajectoryPoint" and topic name "/pva_setpoint".








