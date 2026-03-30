#!/bin/bash

gnome-terminal -- bash -c "cd ~/PX4-Autopilot && make px4_sitl gz_x500_baylands; exec bash"
sleep 2

gnome-terminal -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 1

gnome-terminal -- bash -c "~/QGroundControl-x86_64.AppImage; exec bash"
sleep 3

gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source ~/drone_ws/install/setup.bash && ros2 run controller offboard_takeoff; exec bash"