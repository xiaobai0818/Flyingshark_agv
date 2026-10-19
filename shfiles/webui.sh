#!/bin/bash
# 在香橙派上打开网页控制台。平板或电脑用浏览器访问打印出来的地址。
source /home/orangepi/Flyingshark_agv/shfiles/env.sh
export PYTHONUNBUFFERED=1
if ! rosnode list >/dev/null 2>&1; then
  roscore >/tmp/flyingshark_roscore.log 2>&1 &
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    rosnode list >/dev/null 2>&1 && break
    sleep 0.4
  done
fi
exec rosrun flyingshark_webui webui_node.py "$@"
