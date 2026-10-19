#!/bin/bash
source /home/orangepi/Flyingshark_agv/shfiles/env.sh
exec roslaunch flyingshark_bringup bringup.launch "$@"
