#!/bin/bash
source /home/orangepi/Flyingshark_agv/shfiles/env.sh
exec rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=teleop_cmd_vel
