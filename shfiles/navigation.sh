#!/bin/bash
source /home/orangepi/Flyingshark_agv/shfiles/ensure_drivers.sh
MAP="/home/orangepi/Flyingshark_agv/map/r10.yaml"
if [ ! -f "$MAP" ]; then
  echo "还没有栅格地图 $MAP"
  echo "先运行 bash shfiles/mapping.sh 建图，再在另一个终端运行 bash shfiles/save_map.sh"
  exit 1
fi
ensure_drivers || exit 1
exec roslaunch flyingshark_bringup navigation.launch start_drivers:=false "$@"
