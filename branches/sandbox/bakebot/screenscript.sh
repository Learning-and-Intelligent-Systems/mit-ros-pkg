#!/bin/bash
screen -d -m
screen -X -p 0 title dash
screen -X -p 0 stuff "sudo robot start -e
"
screen -X -p 0 stuff "y
"
screen -X -p 0 stuff "rosrun pr2_dashboard pr2_dashboard
"
screen -X -p 0 screen

screen -X -p 1 title teleop
screen -X -p 1 stuff "roslaunch pr2_teleop teleop_joystick.launch
"
screen -X -p 1 screen

screen -X -p 2 title pr2cm
screen -X -p 2 stuff "rosrun pr2_controller_manager pr2_controller_manager list
"
screen -X -p 2 screen

screen -X -p 3 title src
screen -X -p 3 stuff "cd src
"
