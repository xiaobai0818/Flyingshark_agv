#!/bin/bash
# 五个工作空间按依赖顺序叠加：驱动 -> SLAM -> 导航 -> 工具 -> 网页。
ROOT="/home/orangepi/Flyingshark_agv"
source /opt/ros/noetic/setup.bash
# 后面的工作空间用 --extend，避免后一个 setup 把前面的包路径清掉。
if [ -f "$ROOT/driver/devel/setup.bash" ]; then
  source "$ROOT/driver/devel/setup.bash" --extend
fi
if [ -f "$ROOT/slam/install_isolated/setup.bash" ]; then
  source "$ROOT/slam/install_isolated/setup.bash" --extend
elif [ -f "$ROOT/slam/devel_isolated/setup.bash" ]; then
  source "$ROOT/slam/devel_isolated/setup.bash" --extend
fi
if [ -f "$ROOT/navigation/devel/setup.bash" ]; then
  source "$ROOT/navigation/devel/setup.bash" --extend
fi
if [ -f "$ROOT/utils/devel/setup.bash" ]; then
  source "$ROOT/utils/devel/setup.bash" --extend
fi
if [ -f "$ROOT/ui/devel/setup.bash" ]; then
  source "$ROOT/ui/devel/setup.bash" --extend
fi
export FLYINGSHARK_ROOT="$ROOT"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
# Astra Pro 在 USB2 上需要更大的 usbfs 缓冲。
if [ -w /sys/module/usbcore/parameters/usbfs_memory_mb ]; then
  echo 128 > /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || true
fi
