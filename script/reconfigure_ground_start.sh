#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/cc/ros_ws/mantis_ws/devel/setup.bash

python ground_server.py &
sleep 2
rosrun rqt_reconfigure rqt_reconfigure &
sleep 2
python ground_reconfigure.py

exit 0
