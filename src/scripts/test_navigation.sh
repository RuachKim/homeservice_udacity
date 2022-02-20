#!/bin/bash
echo $PWD
cd ../../;
echo $PWD
catkin_make;

xterm  -e "source devel/setup.bash; export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/map/UdacityOffice.world"; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm  -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" 
