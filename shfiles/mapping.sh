#!/bin/bash
source /home/orangepi/Flyingshark_agv/shfiles/ensure_drivers.sh
ensure_drivers || exit 1
exec roslaunch flyingshark_bringup mapping.launch start_drivers:=false "$@"
