# 被其他脚本 source。底盘、雷达、相机没开时先拉起来，已开则不动。
ensure_drivers() {
  local root="/home/orangepi/Flyingshark_agv"
  # shellcheck disable=SC1090
  source "$root/shfiles/env.sh"
  if ! rosnode list >/dev/null 2>&1; then
    roscore >/tmp/flyingshark_roscore.log 2>&1 &
    local _
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      rosnode list >/dev/null 2>&1 && break
      sleep 0.4
    done
  fi
  if rosnode list 2>/dev/null | grep -q '/ekf_localization'; then
    return 0
  fi
  setsid roslaunch flyingshark_bringup bringup.launch >/tmp/flyingshark_bringup.log 2>&1 &
  echo $! > /tmp/flyingshark_bringup.pid
  local _
  for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
    if rosnode list 2>/dev/null | grep -q '/ekf_localization'; then
      return 0
    fi
    sleep 0.5
  done
  echo "底盘、雷达或相机没有起来，看 /tmp/flyingshark_bringup.log" >&2
  return 1
}
