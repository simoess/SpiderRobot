#!/bin/bash

roscore &
sleep 5s
roslaunch rosbridge_server rosbridge_websocket.launch &# porta 9090
sleep 5s
rosrun servo_master ServoMaster.py &
sleep 5s
rosrun robo_aranha RoboAranha.py &
