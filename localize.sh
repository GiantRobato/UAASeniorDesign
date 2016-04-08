#!/bin/bash

dir=$PWD

killall screen

screen -d -m -S kinect $dir"/startKinect.sh"
echo -n "connecting to kinect sensor"

for i in `seq 1 10`;
do
	echo -n " . "
	sleep 1
done
echo "connected to kinect"
screen -d -m -S rtab $dir"/localizeRtab.sh"
