#!/bin/bash
# 建图过程中在另一个终端执行。会结束当前轨迹并把地图写到 map/。
source /home/orangepi/Flyingshark_agv/shfiles/env.sh
MAP_DIR="/home/orangepi/Flyingshark_agv/map"
mkdir -p "$MAP_DIR"
rosrun map_server map_saver -f "${MAP_DIR}/r10" map:=/map
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${MAP_DIR}/r10.pbstream'}"
echo "已保存 ${MAP_DIR}/r10.pbstream 和 ${MAP_DIR}/r10.pgm / r10.yaml"
