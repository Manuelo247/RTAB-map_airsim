#!/bin/bash
# Pasar DISPLAY_IP como argumento
export DISPLAY_IP=$1
echo $DISPLAY_IP
python3 /home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts/test.py
