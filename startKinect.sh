#!/bin/bash
source /opt/ros/indigo/setup.bash

roslaunch freenect_launch freenect.launch depth_registration:=true
