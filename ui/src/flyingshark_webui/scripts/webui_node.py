#!/usr/bin/env python3
"""Flyingshark web console.

Pose is pushed at 30 Hz. The map is a full-resolution PNG (no resize);
only a changed map is sent. Camera frames are JPEG and slow down when the
outgoing rate would pass 1 MB/s.
"""

import glob
import json
import math
import os
import re
import signal
import struct
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import Empty
from cv_bridge import CvBridge
from actionlib_msgs.msg import GoalID, GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image, LaserScan, PointCloud2

ROOT = "/home/orangepi/Flyingshark_agv"
MAP_DIR = os.path.join(ROOT, "map")
SETTINGS_PATH = os.path.join(MAP_DIR, "webui_settings.json")
ENV = os.path.join(ROOT, "shfiles", "env.sh")
WEB = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "web"))
BANDWIDTH_BUDGET = 1024 * 1024
MAX_V = 0.25
MAX_W = 0.80
POSE_DT = 1.0 / 30.0
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_.-]+\.yaml$")
FUNCTION_HINTS = (
    "move_base",
    "cartographer",
    "amcl",
    "map_server",
)
DRIVER_HINTS = (
    "ekf_localization",
    "openctr",
    "ldlidar",
    "ld06",
    "scan_filter",
    "twist_mux",
    "robot_state_publisher",
    "joint_state_publisher",
    "camera",
    "rgb_cloud",
    "depth_cloud",
    "astra",
)
STACK_HINTS = FUNCTION_HINTS + DRIVER_HINTS


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def clamp_num(value, default, low, high):
    try:
        return max(low, min(high, float(value)))
    except (TypeError, ValueError):
        return default


def default_ui_settings():
    return {
        "teleop_v": 0.25,
        "teleop_w": 0.8,
        "nav_v": 0.35,
        "nav_w": 1.0,
        "keyboard": True,
    }


def load_ui_settings():
    data = default_ui_settings()
    try:
        with open(SETTINGS_PATH, "r", encoding="utf-8") as handle:
            saved = json.load(handle)
    except (OSError, ValueError):
        return data
    if not isinstance(saved, dict):
        return data
    data["teleop_v"] = clamp_num(saved.get("teleop_v", data["teleop_v"]), data["teleop_v"], 0.05, 0.6)
    data["teleop_w"] = clamp_num(saved.get("teleop_w", data["teleop_w"]), data["teleop_w"], 0.2, 1.5)
    data["nav_v"] = clamp_num(saved.get("nav_v", data["nav_v"]), data["nav_v"], 0.05, 0.6)
    data["nav_w"] = clamp_num(saved.get("nav_w", data["nav_w"]), data["nav_w"], 0.2, 1.5)
    if "keyboard" in saved:
        data["keyboard"] = bool(saved["keyboard"])
    return data


def request_system_power(action):
    commands = {
        "reboot": ["/usr/bin/sudo", "-n", "/bin/systemctl", "reboot"],
        "shutdown": ["/usr/bin/sudo", "-n", "/bin/systemctl", "poweroff"],
    }
    cmd = commands.get(action)
    if cmd is None:
        return {"ok": False, "error": "未知系统操作"}
    try:
        proc = subprocess.run(cmd, timeout=8, capture_output=True, text=True)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "error": "系统操作失败: %s" % exc}
    if proc.returncode != 0:
        detail = (proc.stderr or proc.stdout or "系统操作失败").strip()
        return {"ok": False, "error": detail}
    return {"ok": True, "message": "正在重启" if action == "reboot" else "正在关机"}


def astra_depth_present():
    # Astra Pro 彩色是 0501，深度是另一颗芯片，常见 0403。
    depth_ids = {"0401", "0402", "0403", "0404", "0407", "060b", "060e", "060f", "0610"}
    for vendor_path in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        folder = os.path.dirname(vendor_path)
        try:
            with open(vendor_path, "r", encoding="utf-8") as handle:
                vendor = handle.read().strip()
            with open(os.path.join(folder, "idProduct"), "r", encoding="utf-8") as handle:
                product = handle.read().strip()
        except OSError:
            continue
        if vendor == "2bc5" and product in depth_ids:
            return True
    return False


def list_maps():
    names = []
    for path in sorted(glob.glob(os.path.join(MAP_DIR, "*.yaml"))):
        name = os.path.basename(path)
        if name == "blank.yaml":
            continue
        names.append(name)
    return names


class Hub(object):
    def __init__(self):
        self.cv = threading.Condition()
        self.pose = None
        self.pose_seq = 0
        self.state = {}
        self.state_seq = 0
        self.map_meta = None
        self.map_seq = 0
        self.map_png = b""
        self.cloud = None
        self.cloud_seq = 0
        self.rgb_meta = None
        self.rgb_blob = b""
        self.rgb_seq = 0
        self.paths = None
        self.paths_seq = 0
        self.costs = {}
        self.jpeg = b""
        self.jpeg_seq = 0
        self.bytes = []
        self.lock = threading.Lock()

    def note(self, nbytes):
        now = time.time()
        with self.lock:
            self.bytes.append((now, nbytes))
            cutoff = now - 1.0
            while self.bytes and self.bytes[0][0] < cutoff:
                self.bytes.pop(0)

    def bandwidth(self):
        now = time.time()
        with self.lock:
            return sum(n for t, n in self.bytes if now - t <= 1.0)

    def set_pose(self, pose):
        with self.cv:
            self.pose = pose
            self.pose_seq += 1
            self.cv.notify_all()

    def set_state(self, state):
        with self.cv:
            self.state = state
            self.state_seq += 1
            self.cv.notify_all()

    def set_map(self, meta, png):
        with self.cv:
            self.map_seq += 1
            meta = dict(meta)
            meta["rev"] = self.map_seq
            self.map_meta = meta
            self.map_png = png
            self.cv.notify_all()

    def set_cloud(self, cloud):
        with self.cv:
            self.cloud = cloud
            self.cloud_seq += 1
            self.cv.notify_all()

    def set_paths(self, paths):
        with self.cv:
            self.paths = paths
            self.paths_seq += 1
            self.cv.notify_all()

    def set_cost(self, kind, meta, png):
        with self.cv:
            slot = self.costs.get(kind) or {"seq": 0}
            slot["seq"] = slot["seq"] + 1
            meta = dict(meta)
            meta["rev"] = slot["seq"]
            meta["kind"] = kind
            slot["meta"] = meta
            slot["png"] = png
            self.costs[kind] = slot
            self.cv.notify_all()

    def set_rgb(self, meta, blob):
        with self.cv:
            self.rgb_seq += 1
            meta = dict(meta or {})
            meta["rev"] = self.rgb_seq
            self.rgb_meta = meta
            self.rgb_blob = blob or b""
            self.cv.notify_all()

    def clear_scene(self):
        with self.cv:
            self.map_seq += 1
            self.map_meta = {"clear": True, "rev": self.map_seq}
            self.map_png = b""
            self.cloud_seq += 1
            self.cloud = {"frame": "", "laser": [], "depth": []}
            self.rgb_seq += 1
            self.rgb_meta = {"clear": True, "rev": self.rgb_seq}
            self.rgb_blob = b""
            self.paths_seq += 1
            self.paths = {"frame": "", "global": [], "local": []}
            for kind in ("global", "local"):
                slot = self.costs.get(kind) or {"seq": 0}
                slot["seq"] = slot.get("seq", 0) + 1
                slot["png"] = b""
                slot["meta"] = {"clear": True, "kind": kind, "rev": slot["seq"]}
                self.costs[kind] = slot
            self.cv.notify_all()

    def clear_map_layers(self):
        with self.cv:
            self.map_seq += 1
            self.map_meta = {"clear": True, "rev": self.map_seq}
            self.map_png = b""
            self.paths_seq += 1
            self.paths = {"frame": "", "global": [], "local": []}
            for kind in ("global", "local"):
                slot = self.costs.get(kind) or {"seq": 0}
                slot["seq"] = slot.get("seq", 0) + 1
                slot["png"] = b""
                slot["meta"] = {"clear": True, "kind": kind, "rev": slot["seq"]}
                self.costs[kind] = slot
            self.cv.notify_all()

    def set_jpeg(self, blob):
        with self.lock:
            self.jpeg = blob
            self.jpeg_seq += 1

    def get_jpeg(self):
        with self.lock:
            return self.jpeg, self.jpeg_seq


class Stack(object):
    def __init__(self, hub):
        self.hub = hub
        self.lock = threading.Lock()
        self.op_lock = threading.Lock()
        self.mode = "idle"
        self.busy = False
        self.busy_action = ""
        self.message = "未启动"
        self.map_name = ""
        self.proc = None
        self.driver_proc = None
        self.watch = threading.Thread(target=self._watch, daemon=True)
        self.watch.start()
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self._nav_result, queue_size=1)

    def _nav_result(self, msg):
        status = msg.status.status
        if status == GoalStatus.SUCCEEDED:
            text = "已到达导航点"
        elif status == GoalStatus.PREEMPTED:
            text = "导航已取消"
        elif status == GoalStatus.ABORTED:
            text = "规划不出路径，车没有动。蓝车要在空地上；请再点重定位，或在地图上点一下车的真实位置"
        else:
            return
        with self.lock:
            self.message = text
        self.push()

    def snapshot(self):
        with self.lock:
            state = {
                "mode": self.mode,
                "busy": self.busy,
                "busy_action": self.busy_action,
                "message": self.message,
                "map_name": self.map_name,
                "maps": list_maps(),
                "bandwidth": self.hub.bandwidth(),
                "robot_radius": 0.108,
                "lidar_x": 0.06,
                "camera_x": 0.055,
                "wheel_y": 0.082,
                "wheel_radius": 0.037,
            }
            bridge = getattr(self, "bridge", None)
            if bridge is not None:
                state["settings"] = bridge.settings_payload()
            return state

    def push(self):
        self.hub.set_state(self.snapshot())

    def _watch(self):
        while not rospy.is_shutdown():
            time.sleep(0.5)
            with self.lock:
                proc = self.proc
                busy = self.busy
                mode = self.mode
            with self.lock:
                driver = self.driver_proc
            if not busy and driver is not None and driver.poll() is not None and mode != "idle":
                with self.lock:
                    if self.driver_proc is driver and not self.busy:
                        self.driver_proc = None
                        self.proc = None
                        self.mode = "idle"
                        self.map_name = ""
                        self.message = "传感器已退出（%s）" % driver.returncode
                        self._suspend_scene()
                self.push()
                continue
            if busy or proc is None or mode in ("idle", "init"):
                continue
            if proc.poll() is not None:
                drivers_up = self._have_drivers()
                with self.lock:
                    if self.proc is proc and not self.busy:
                        self.proc = None
                        if drivers_up:
                            self.mode = "init"
                            self.map_name = ""
                            self.message = "功能已退出（%s），传感器仍在运行" % proc.returncode
                            self._clear_map_layers()
                        else:
                            self.mode = "idle"
                            self.map_name = ""
                            self.message = "程序已退出（%s）" % proc.returncode
                            self._suspend_scene()
                self.push()

    def handle(self, op, body):
        if op == "delete_map":
            return self._delete_map(body.get("map") or "")
        if op == "reloc_global":
            return self._global_reloc()
        if op == "reloc_stop":
            self.bridge.stop_reloc_spin()
            with self.lock:
                self.message = "已停止转动，可以发导航点"
            self.push()
            return {"ok": True, "state": self.snapshot()}
        if op == "init":
            if not self.op_lock.acquire(blocking=False):
                return {"ok": False, "error": "正在切换，请稍候", "state": self.snapshot()}
            try:
                if body.get("on"):
                    return self._start_drivers()
                return self._stop_all()
            finally:
                self.op_lock.release()
        if op in ("blank", "mapping", "nav", "save"):
            if not self.op_lock.acquire(blocking=False):
                return {"ok": False, "error": "正在切换，请稍候", "state": self.snapshot()}
            try:
                if op == "save":
                    return self._save()
                turn_on = bool(body.get("on"))
                if op == "nav" and turn_on:
                    return self._set_mode("nav", body.get("map") or "")
                if not turn_on:
                    return self._stop_function_keep_drivers()
                if op == "blank":
                    return self._set_mode("blank")
                if op == "mapping":
                    return self._set_mode("mapping")
                return self._set_mode("nav", body.get("map") or "")
            finally:
                self.op_lock.release()
        return {"ok": False, "error": "未知操作", "state": self.snapshot()}

    def _global_reloc(self):
        if self.mode != "nav":
            return {"ok": False, "error": "请先开启已知地图导航", "state": self.snapshot()}
        ok, message = self.bridge.global_reloc()
        with self.lock:
            self.message = message
        self.push()
        state = self.snapshot()
        if not ok:
            return {"ok": False, "error": message, "state": state}
        return {"ok": True, "state": state}

    def _delete_map(self, map_name):
        if not MAP_NAME_RE.match(map_name or "") or map_name == "blank.yaml":
            return {"ok": False, "error": "不能删除这张地图", "state": self.snapshot()}
        with self.lock:
            busy = self.busy
            navigating = self.mode == "nav" and self.map_name == map_name
        if busy:
            return {"ok": False, "error": "正在切换，请稍候", "state": self.snapshot()}
        if navigating:
            return {"ok": False, "error": "这张地图正在导航，请先关掉导航", "state": self.snapshot()}
        root = os.path.realpath(MAP_DIR)
        yaml_path = os.path.realpath(os.path.join(MAP_DIR, map_name))
        if not yaml_path.startswith(root + os.sep) or not os.path.isfile(yaml_path):
            return {"ok": False, "error": "地图文件不存在", "state": self.snapshot()}
        stem = os.path.splitext(map_name)[0]
        targets = []
        for path in (yaml_path, os.path.join(MAP_DIR, stem + ".pgm"), os.path.join(MAP_DIR, stem + ".pbstream")):
            real = os.path.realpath(path)
            if real.startswith(root + os.sep) and os.path.isfile(real) and real not in targets:
                targets.append(real)
        try:
            with open(yaml_path, "r", encoding="utf-8") as handle:
                for line in handle:
                    if not line.startswith("image:"):
                        continue
                    image = line.split(":", 1)[1].strip()
                    image_path = image if os.path.isabs(image) else os.path.join(os.path.dirname(yaml_path), image)
                    real = os.path.realpath(image_path)
                    if real.startswith(root + os.sep) and os.path.isfile(real) and real not in targets:
                        targets.append(real)
                    break
        except OSError:
            pass
        try:
            for path in targets:
                os.remove(path)
        except OSError as exc:
            return {"ok": False, "error": "删除失败: %s" % exc, "state": self.snapshot()}
        with self.lock:
            self.message = "已删除 " + map_name
        self.push()
        return {"ok": True, "state": self.snapshot()}

    def _have_drivers(self):
        with self.lock:
            proc = self.driver_proc
        if proc is not None and proc.poll() is None:
            return True
        return bool(self._nodes_matching(DRIVER_HINTS))

    def _start_drivers(self):
        if self._have_drivers():
            with self.lock:
                self.mode = "init" if self.mode == "idle" else self.mode
                self.message = "传感器已在运行"
            self._resume_scene()
            self.push()
            return {"ok": True, "state": self.snapshot()}
        with self.lock:
            self.busy = True
            self.busy_action = "init"
            self.message = "正在打开雷达、相机和底盘…"
        self.push()
        error = ""
        try:
            proc = self._spawn("init", "")
            time.sleep(1.2)
            if proc.poll() is not None:
                error = self._log_tail() or "初始化失败"
                with self.lock:
                    self.mode = "idle"
                    self.driver_proc = None
                    self.message = error
            else:
                with self.lock:
                    self.driver_proc = proc
                    self.mode = "init"
                    self.map_name = ""
                    self.message = "雷达、相机和底盘已打开"
                    if not astra_depth_present():
                        self.message += "。深度相机没出现在 USB 上，请把那根线插到 USB 3 口"
                self._resume_scene()
        except Exception as exc:
            error = str(exc)
            with self.lock:
                self.mode = "idle"
                self.message = error
        finally:
            with self.lock:
                self.busy = False
                self.busy_action = ""
            self.push()
        state = self.snapshot()
        if error:
            return {"ok": False, "error": error, "state": state}
        return {"ok": True, "state": state}

    def _stop_all(self):
        with self.lock:
            self.busy = True
            self.busy_action = "init"
            self.message = "正在关闭…"
        self.push()
        error = ""
        try:
            self._suspend_scene()
            self._stop_drivers()
            with self.lock:
                self.mode = "idle"
                self.map_name = ""
                self.message = "已关闭，画面已清除"
        except Exception as exc:
            error = str(exc)
            with self.lock:
                self.message = error
        finally:
            with self.lock:
                self.busy = False
                self.busy_action = ""
            self.push()
        state = self.snapshot()
        if error:
            return {"ok": False, "error": error, "state": state}
        return {"ok": True, "state": state}

    def _stop_function_keep_drivers(self):
        with self.lock:
            self.busy = True
            self.busy_action = "init"
            self.message = "正在关闭功能…"
        self.push()
        error = ""
        try:
            self._stop_function()
            self._clear_map_layers()
            drivers_up = self._have_drivers()
            with self.lock:
                if drivers_up:
                    self.mode = "init"
                    self.map_name = ""
                    self.message = "功能已关闭，传感器仍在运行"
                else:
                    self.mode = "idle"
                    self.map_name = ""
                    self.message = "已关闭"
                    self._suspend_scene()
        except Exception as exc:
            error = str(exc)
            with self.lock:
                self.message = error
        finally:
            with self.lock:
                self.busy = False
                self.busy_action = ""
            self.push()
        state = self.snapshot()
        if error:
            return {"ok": False, "error": error, "state": state}
        return {"ok": True, "state": state}

    def _set_mode(self, mode, map_name=""):
        if not self._have_drivers():
            return {"ok": False, "error": "请先点击初始化", "state": self.snapshot()}
        if mode == "nav":
            if not MAP_NAME_RE.match(map_name or ""):
                return {"ok": False, "error": "请先选择一张地图", "state": self.snapshot()}
            map_path = os.path.realpath(os.path.join(MAP_DIR, map_name))
            if not map_path.startswith(os.path.realpath(MAP_DIR) + os.sep) or not os.path.isfile(map_path):
                return {"ok": False, "error": "地图文件不存在", "state": self.snapshot()}
        else:
            map_path = ""
        with self.lock:
            self.busy = True
            self.busy_action = mode
            self.message = "正在切换…"
        self.push()
        error = ""
        try:
            with self.lock:
                self.message = "正在关闭上一个功能…"
            self.push()
            self._stop_function()
            self._clear_map_layers()
            proc = self._spawn(mode, map_path)
            time.sleep(1.2)
            if proc.poll() is not None:
                error = self._log_tail() or "启动失败"
                drivers_up = self._have_drivers()
                with self.lock:
                    self.proc = None
                    self.mode = "init" if drivers_up else "idle"
                    self.message = error
            else:
                with self.lock:
                    self.proc = proc
                    self.mode = mode
                    self.map_name = os.path.basename(map_path) if map_path else ""
                    self.message = {
                        "blank": "无地图导航已开启",
                        "mapping": "正在建图",
                        "nav": "已知地图导航已开启",
                    }.get(mode, "已开启")
                    if mode in ("blank", "mapping", "nav") and not astra_depth_present():
                        self.message += "。深度相机没出现在 USB 上，请把那根线插到 USB 3 口"
                if mode in ("blank", "nav"):
                    threading.Timer(1.5, self.bridge.apply_nav_speed).start()
        except Exception as exc:
            error = str(exc)
            drivers_up = self._have_drivers()
            with self.lock:
                self.mode = "init" if drivers_up else "idle"
                self.message = error
        finally:
            with self.lock:
                self.busy = False
                self.busy_action = ""
            self.push()
        state = self.snapshot()
        if error:
            return {"ok": False, "error": error, "state": state}
        return {"ok": True, "state": state}

    def _spawn(self, mode, map_path):
        if mode == "init":
            launch = "roslaunch flyingshark_bringup bringup.launch"
        elif mode == "blank":
            launch = "roslaunch flyingshark_bringup blank_nav.launch open_rviz:=false start_drivers:=false"
        elif mode == "mapping":
            launch = "roslaunch flyingshark_bringup mapping.launch open_rviz:=false start_drivers:=false"
        elif mode == "nav":
            launch = "roslaunch flyingshark_bringup navigation.launch open_rviz:=false start_drivers:=false map_file:=%s" % map_path
        else:
            raise RuntimeError("未知模式")
        log_path = "/tmp/flyingshark_webui_%s.log" % mode
        self._last_log = log_path
        logf = open(log_path, "wb")
        cmd = "source %s && exec %s" % (ENV, launch)
        return subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=logf,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            cwd=ROOT,
        )

    def _suspend_scene(self):
        bridge = getattr(self, "bridge", None)
        if bridge is not None:
            bridge.suspend_scene()
        else:
            self.hub.clear_scene()

    def _resume_scene(self):
        bridge = getattr(self, "bridge", None)
        if bridge is not None:
            bridge.resume_scene()

    def _clear_map_layers(self):
        bridge = getattr(self, "bridge", None)
        if bridge is not None:
            bridge.clear_map_layers()
        else:
            self.hub.clear_map_layers()

    def _stop_proc(self, proc):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=3)

    def _stop_function(self):
        with self.lock:
            proc = self.proc
            self.proc = None
        self._stop_proc(proc)
        for pattern in (
            "roslaunch flyingshark_bringup mapping.launch",
            "roslaunch flyingshark_bringup blank_nav.launch",
            "roslaunch flyingshark_bringup navigation.launch",
        ):
            subprocess.run(["pkill", "-INT", "-f", pattern], timeout=3, check=False)
        self._kill_nodes(self._nodes_matching(FUNCTION_HINTS))
        self._wait_until_clear(FUNCTION_HINTS, 6)

    def _stop_drivers(self):
        self._stop_function()
        with self.lock:
            proc = self.driver_proc
            self.driver_proc = None
        self._stop_proc(proc)
        subprocess.run(["pkill", "-INT", "-f", "roslaunch flyingshark_bringup bringup.launch"], timeout=3, check=False)
        self._kill_nodes(self._nodes_matching(DRIVER_HINTS))
        self._wait_until_clear(DRIVER_HINTS, 8)
        left = self._nodes_matching(DRIVER_HINTS)
        if left:
            subprocess.run(["pkill", "-KILL", "-f", "roslaunch flyingshark_bringup bringup.launch"], timeout=3, check=False)
            self._kill_nodes(left)
            self._wait_until_clear(DRIVER_HINTS, 6)
        left = self._nodes_matching(STACK_HINTS)
        if left:
            raise RuntimeError("还有进程没退出: %s" % " ".join(left[:6]))

    def _nodes_matching(self, hints):
        try:
            out = subprocess.check_output(["rosnode", "list"], stderr=subprocess.DEVNULL, timeout=5, text=True)
        except Exception:
            return []
        names = []
        for line in out.splitlines():
            name = line.strip()
            if not name or name in ("/rosout", "/flyingshark_webui"):
                continue
            low = name.lower()
            if any(hint in low for hint in hints):
                names.append(name)
        return names

    def _kill_nodes(self, names):
        if not names:
            return
        subprocess.run(
            ["rosnode", "kill"] + names,
            timeout=20,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _wait_until_clear(self, hints, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline and self._nodes_matching(hints):
            time.sleep(0.3)

    def _log_tail(self):
        path = getattr(self, "_last_log", "/tmp/flyingshark_webui_init.log")
        try:
            with open(path, "r", errors="replace") as handle:
                lines = handle.readlines()[-8:]
            text = " ".join(line.strip() for line in lines if line.strip())
            return text[-240:]
        except OSError:
            return ""

    def _save(self):
        with self.lock:
            if self.mode != "mapping":
                return {"ok": False, "error": "请先打开手动建图", "state": self.snapshot()}
            self.busy = True
            self.busy_action = "save"
            self.message = "正在保存地图…"
        self.push()
        error = ""
        try:
            os.makedirs(MAP_DIR, exist_ok=True)
            prefix = os.path.join(MAP_DIR, "r10")
            saver = subprocess.run(
                ["rosrun", "map_server", "map_saver", "-f", prefix, "map:=/map"],
                timeout=40,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            if not os.path.isfile(prefix + ".yaml"):
                error = (saver.stdout or "保存栅格地图失败")[-200:]
            else:
                pb = prefix + ".pbstream"
                writer = subprocess.run(
                    ["rosservice", "call", "/write_state", "{filename: '%s'}" % pb],
                    timeout=40,
                    check=False,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
                if writer.returncode == 0 and os.path.isfile(pb):
                    message = "已保存 r10.yaml 和 r10.pbstream，建图可以继续"
                else:
                    message = "已保存 r10.yaml。pbstream 未写上，已知地图导航仍可用这张栅格图"
                with self.lock:
                    self.message = message
                    self.map_name = "r10.yaml"
        except subprocess.TimeoutExpired:
            error = "保存超时，请确认地图话题有数据"
        except Exception as exc:
            error = str(exc)
        finally:
            with self.lock:
                self.busy = False
                self.busy_action = ""
                if error:
                    self.message = error
            self.push()
        state = self.snapshot()
        if error:
            return {"ok": False, "error": error, "state": state}
        return {"ok": True, "state": state}


class Bridge(object):
    def __init__(self, hub):
        self.hub = hub
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cmd_pub = rospy.Publisher("teleop_cmd_vel", Twist, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self._reloc_stop = False
        self._reloc_spin = None
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.frame = None
        self.frame_seq = 0
        self.scan = None
        self.depth = None
        self._rgb_time = 0.0
        self.paths = {"global": [], "local": []}
        self.cost_pending = {}
        self.cost_event = threading.Event()
        self.cost_lock = threading.Lock()
        self.tick = 0
        self.cmd = Twist()
        saved = load_ui_settings()
        self.teleop_v = saved["teleop_v"]
        self.teleop_w = saved["teleop_w"]
        self.nav_v = saved["nav_v"]
        self.nav_w = saved["nav_w"]
        self.keyboard = saved["keyboard"]
        self._nav_clients = {}
        self.cmd_stamp = 0.0
        self.cmd_lock = threading.Lock()
        self.zero_sent = True
        self.publish_scene = False
        self.scene_gen = 0
        self.map_crc = None
        self.map_event = threading.Event()
        self.map_pending = None
        self.map_lock = threading.Lock()
        self.jpeg_quality = 50
        self.cam_interval = 1.0 / 12.0
        rospy.Subscriber("/camera/audio/left", Int16MultiArray, self._audio_left, queue_size=1)
        rospy.Subscriber("/camera/audio/right", Int16MultiArray, self._audio_right, queue_size=1)
        self.audio_level = {"left": 0.0, "right": 0.0}
        self.audio_time = {"left": 0.0, "right": 0.0}
        self.audio_lock = threading.Lock()
        rospy.Subscriber("/camera/color/image_raw", Image, self._image, queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber("/map", OccupancyGrid, self._map, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=1, buff_size=2 ** 20)
        rospy.Subscriber("/camera/depth_obstacles", PointCloud2, self._depth, queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber("/camera/rgb_points", PointCloud2, self._rgb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self._global_path, queue_size=1)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self._local_path, queue_size=1)
        rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self._local_path, queue_size=1)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self._global_cost, queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self._local_cost, queue_size=1, buff_size=2 ** 20)
        rospy.Timer(rospy.Duration(POSE_DT), self._tick)
        threading.Thread(target=self._encode_camera, daemon=True).start()
        threading.Thread(target=self._encode_map, daemon=True).start()
        threading.Thread(target=self._encode_cost, daemon=True).start()

    def suspend_scene(self):
        self.publish_scene = False
        self.scene_gen += 1
        self.map_crc = None
        self.scan = None
        self.depth = None
        self.paths = {"global": [], "local": []}
        with self.map_lock:
            self.map_pending = None
        with self.cost_lock:
            self.cost_pending = {}
        self.hub.clear_scene()

    def resume_scene(self):
        self.map_crc = None
        self.publish_scene = True

    def clear_map_layers(self):
        self.map_crc = None
        self.paths = {"global": [], "local": []}
        with self.map_lock:
            self.map_pending = None
        with self.cost_lock:
            self.cost_pending = {}
        self.hub.clear_map_layers()

    def _image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "图像转换失败: %s", exc)
            return
        self.frame = image
        self.frame_seq += 1

    def _scan(self, msg):
        self.scan = msg
        self._push_cloud("laser")

    def _depth(self, msg):
        self.depth = msg
        self._push_cloud("depth")

    def _global_path(self, msg):
        self._push_path("global", msg)

    def _local_path(self, msg):
        self._push_path("local", msg)

    def _global_cost(self, msg):
        self._queue_cost("global", msg)

    def _local_cost(self, msg):
        self._queue_cost("local", msg)

    def _map(self, msg):
        if not self.publish_scene:
            return
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        arr = np.asarray(msg.data, dtype=np.int16)
        if arr.size != msg.info.width * msg.info.height:
            return
        info = (
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            yaw_from_quat(msg.info.origin.orientation),
        )
        crc = zlib.crc32(arr.tobytes()) ^ zlib.crc32(repr(info).encode())
        with self.map_lock:
            if crc == self.map_crc:
                return
            self.map_crc = crc
            self.map_pending = (arr, info, self.scene_gen)
            self.map_event.set()

    def _encode_map(self):
        while not rospy.is_shutdown():
            self.map_event.wait(0.5)
            self.map_event.clear()
            with self.map_lock:
                pending = self.map_pending
                self.map_pending = None
            if not pending:
                continue
            arr, info, gen = pending
            if gen != self.scene_gen or not self.publish_scene:
                continue
            width, height, resolution, origin_x, origin_y, origin_yaw = info
            grid = np.flipud(arr.reshape((height, width)))
            image = np.full((height, width), 96, np.uint8)
            known = grid >= 0
            occ = np.clip(grid[known], 0, 100).astype(np.int32)
            image[known] = (235 - (occ * 205) // 100).astype(np.uint8)
            ok, encoded = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 6])
            if not ok:
                continue
            png = encoded.tobytes()
            if not png:
                continue
            self.hub.set_map(
                {
                    "width": width,
                    "height": height,
                    "resolution": resolution,
                    "origin_x": origin_x,
                    "origin_y": origin_y,
                    "yaw": origin_yaw,
                },
                png,
            )
            rospy.loginfo("地图 %dx%d，PNG %d KB，未缩放", width, height, len(png) // 1024)

    def _queue_cost(self, kind, msg):
        if not self.publish_scene:
            return
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        arr = np.asarray(msg.data, dtype=np.int16)
        if arr.size != msg.info.width * msg.info.height:
            return
        pending = (
            arr,
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            msg.header.frame_id or "map",
            yaw_from_quat(msg.info.origin.orientation),
            self.scene_gen,
        )
        with self.cost_lock:
            self.cost_pending[kind] = pending
        self.cost_event.set()

    def _encode_cost(self):
        while not rospy.is_shutdown():
            self.cost_event.wait(0.2)
            self.cost_event.clear()
            with self.cost_lock:
                pending = self.cost_pending
                self.cost_pending = {}
            for kind, item in pending.items():
                if item[-1] != self.scene_gen or not self.publish_scene:
                    continue
                encoded = self._cost_png(item)
                if encoded is None:
                    continue
                meta, png = encoded
                self.hub.set_cost(kind, meta, png)

    def _cost_png(self, item):
        arr, width, height, resolution, origin_x, origin_y, src, origin_yaw, _gen = item
        display = self._display_frame()
        if display is None:
            return None
        placed = self._place_grid(display, src, origin_x, origin_y, origin_yaw)
        if placed is None:
            return None
        grid = np.flipud(arr.reshape((height, width)))
        image = np.zeros((height, width, 4), np.uint8)
        lethal = grid >= 99
        mid = (grid > 0) & (grid < 99)
        image[mid] = (50, 150, 230, 120)
        image[lethal] = (40, 50, 220, 210)
        ok, encoded = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            return None
        png = encoded.tobytes()
        if not png:
            return None
        meta = {
            "frame": display,
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin_x": placed[0],
            "origin_y": placed[1],
            "yaw": placed[2],
        }
        return meta, png

    def _encode_camera(self):
        last_seq = -1
        next_time = 0.0
        while not rospy.is_shutdown():
            now = time.time()
            delay = next_time - now
            if delay > 0:
                time.sleep(min(delay, 0.05))
                continue
            frame = self.frame
            seq = self.frame_seq
            if frame is None or seq == last_seq:
                time.sleep(0.02)
                continue
            used = self.hub.bandwidth()
            if used > BANDWIDTH_BUDGET:
                self.jpeg_quality = max(28, self.jpeg_quality - 4)
                self.cam_interval = min(0.2, self.cam_interval + 0.02)
            elif used < int(BANDWIDTH_BUDGET * 0.45):
                self.jpeg_quality = min(62, self.jpeg_quality + 1)
                self.cam_interval = max(1.0 / 15.0, self.cam_interval - 0.005)
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg_quality)]
            )
            if ok:
                self.hub.set_jpeg(encoded.tobytes())
                last_seq = seq
            next_time = time.time() + self.cam_interval

    def _audio_level(self, msg):
        samples = msg.data
        count = len(samples)
        if count == 0:
            return 0.0
        energy = 0.0
        for sample in samples:
            energy += float(sample) * float(sample)
        rms = math.sqrt(energy / count) / 32768.0
        if rms < 1e-5:
            return 0.0
        db = 20.0 * math.log10(rms)
        return max(0.0, min(1.0, (db + 50.0) / 50.0))

    def _remember_audio(self, channel, msg):
        level = self._audio_level(msg)
        with self.audio_lock:
            self.audio_level[channel] = level
            self.audio_time[channel] = time.time()

    def _audio_left(self, msg):
        self._remember_audio("left", msg)

    def _audio_right(self, msg):
        self._remember_audio("right", msg)

    def _audio_payload(self):
        now = time.time()
        with self.audio_lock:
            left = self.audio_level["left"] if now - self.audio_time["left"] < 0.4 else 0.0
            right = self.audio_level["right"] if now - self.audio_time["right"] < 0.4 else 0.0
        return {"audio_l": round(left, 3), "audio_r": round(right, 3)}

    def _tick(self, _event):
        self.tick += 1
        self._publish_cmd()
        pose = self._pose_payload()
        pose["bandwidth"] = self.hub.bandwidth()
        self.hub.set_pose(pose)

    def _publish_cmd(self):
        with self.cmd_lock:
            age = time.time() - self.cmd_stamp
            cmd = Twist()
            cmd.linear.x = self.cmd.linear.x
            cmd.angular.z = self.cmd.angular.z
            if age > 0.3:
                cmd = Twist()
        moving = abs(cmd.linear.x) + abs(cmd.angular.z) > 1e-4
        if not moving and self.zero_sent:
            return
        self.cmd_pub.publish(cmd)
        self.zero_sent = not moving

    def set_cmd(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = max(-self.teleop_v, min(self.teleop_v, float(linear)))
        cmd.angular.z = max(-self.teleop_w, min(self.teleop_w, float(angular)))
        with self.cmd_lock:
            self.cmd = cmd
            self.cmd_stamp = time.time()
        self.zero_sent = False

    def set_speeds(self, body):
        def num(key, default, low, high):
            try:
                return max(low, min(high, float(body.get(key, default))))
            except (TypeError, ValueError):
                return default
        self.teleop_v = num("teleop_v", self.teleop_v, 0.05, 0.6)
        self.teleop_w = num("teleop_w", self.teleop_w, 0.2, 1.5)
        self.nav_v = num("nav_v", self.nav_v, 0.05, 0.6)
        self.nav_w = num("nav_w", self.nav_w, 0.2, 1.5)
        if "keyboard" in body:
            self.keyboard = bool(body.get("keyboard"))
        self.apply_nav_speed()
        self._persist_settings()
        return self.settings_payload()

    def settings_payload(self):
        return {
            "teleop_v": self.teleop_v,
            "teleop_w": self.teleop_w,
            "nav_v": self.nav_v,
            "nav_w": self.nav_w,
            "keyboard": self.keyboard,
        }

    def _persist_settings(self):
        payload = self.settings_payload()
        tmp = SETTINGS_PATH + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
                handle.write("\n")
            os.replace(tmp, SETTINGS_PATH)
        except OSError as exc:
            rospy.logwarn("保存界面设置失败: %s", exc)

    def apply_nav_speed(self):
        try:
            from dynamic_reconfigure.client import Client
        except ImportError:
            return
        config = {"max_vel_x": self.nav_v, "max_vel_theta": self.nav_w}
        for name in ("/move_base/TebLocalPlannerROS", "/move_base/DWAPlannerROS"):
            try:
                client = self._nav_clients.get(name)
                if client is None:
                    client = Client(name, timeout=0.4)
                    self._nav_clients[name] = client
                client.update_configuration(config)
            except Exception:
                self._nav_clients.pop(name, None)

    def publish_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation = quat_from_yaw(float(yaw))
        self.goal_pub.publish(msg)

    def publish_reloc(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = quat_from_yaw(float(yaw))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.pose_pub.publish(msg)
        self._amcl_service("/request_nomotion_update")
        self.clear_costmaps()
        rospy.Timer(rospy.Duration(0.6), lambda _event: self.clear_costmaps(), oneshot=True)

    def clear_costmaps(self):
        self._amcl_service("/move_base/clear_costmaps")

    def global_reloc(self):
        ok, detail = self._amcl_service("/global_localization")
        if not ok:
            return False, "重定位失败：" + detail
        self.clear_costmaps()
        self._amcl_service("/request_nomotion_update")
        self._start_reloc_spin()
        return True, "正在原地慢转一圈，用雷达点云和地图自动对齐"

    def _hold_cmd(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        with self.cmd_lock:
            self.cmd = cmd
            self.cmd_stamp = time.time()
        self.zero_sent = False

    def stop_reloc_spin(self):
        self._reloc_stop = True
        self._hold_cmd(0.0, 0.0)
        self.clear_costmaps()

    def _start_reloc_spin(self):
        spinning = self._reloc_spin is not None and self._reloc_spin.is_alive()
        if spinning:
            return
        self._reloc_stop = False
        self._reloc_spin = threading.Thread(target=self._spin_for_reloc, daemon=True)
        self._reloc_spin.start()

    def _spin_for_reloc(self):
        self.cancel_pub.publish(GoalID())
        # 约一圈。激光每转过一段就更新一次粒子，错误位置会被滤掉。
        end = time.time() + 14.0
        next_update = 0.0
        while time.time() < end and not rospy.is_shutdown() and not self._reloc_stop:
            self._hold_cmd(0.0, 0.45)
            now = time.time()
            if now >= next_update:
                self._amcl_service("/request_nomotion_update")
                next_update = now + 1.0
            if self._reloc_stop:
                break
            time.sleep(0.1)
        self._hold_cmd(0.0, 0.0)
        if not self._reloc_stop:
            self.clear_costmaps()
            rospy.Timer(rospy.Duration(0.8), lambda _event: self.clear_costmaps(), oneshot=True)

    def _amcl_service(self, name):
        try:
            rospy.wait_for_service(name, timeout=1.2)
            rospy.ServiceProxy(name, Empty)()
            return True, ""
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, str(exc)

    def _pose_payload(self):
        looked = self._lookup_pose()
        if looked is None:
            payload = {"ok": False, "hz": round(1.0 / POSE_DT)}
            payload.update(self._audio_payload())
            return payload
        frame, trans = looked
        rot = trans.transform.rotation
        pos = trans.transform.translation
        payload = {
            "ok": True,
            "frame": frame,
            "x": pos.x,
            "y": pos.y,
            "yaw": yaw_from_quat(rot),
            "hz": round(1.0 / POSE_DT),
        }
        payload.update(self._audio_payload())
        return payload

    def _lookup_pose(self):
        for frame in ("map", "odom"):
            try:
                return frame, self.tf_buffer.lookup_transform(frame, "base_footprint", rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        return None

    def _display_frame(self):
        looked = self._lookup_pose()
        if looked is None:
            return None
        return looked[0]

    def _frame_xf(self, target, source):
        if not target or not source:
            return None
        if target == source:
            return 0.0, 0.0, 0.0, 0.0
        try:
            trans = self.tf_buffer.lookup_transform(target, source, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        yaw = yaw_from_quat(trans.transform.rotation)
        pos = trans.transform.translation
        return yaw, pos.x, pos.y, pos.z

    def _apply_xy(self, xf, xs, ys):
        yaw, tx, ty = xf[0], xf[1], xf[2]
        ct, st = math.cos(yaw), math.sin(yaw)
        return tx + ct * xs - st * ys, ty + st * xs + ct * ys

    def _apply_xyz(self, xf, xs, ys, zs):
        yaw, tx, ty, tz = xf
        ct, st = math.cos(yaw), math.sin(yaw)
        return tx + ct * xs - st * ys, ty + st * xs + ct * ys, tz + zs

    def _place_grid(self, display, source, origin_x, origin_y, origin_yaw):
        xf = self._frame_xf(display, source)
        if xf is None:
            return None
        ox, oy = self._apply_xy(xf, origin_x, origin_y)
        return float(ox), float(oy), xf[0] + origin_yaw

    def _push_cloud(self, kind):
        if not self.publish_scene:
            return
        frame = self._display_frame()
        if frame is None:
            return
        if kind == "laser":
            points = self._laser_points(frame)
            key = "laser"
        else:
            points = self._depth_points(frame)
            key = "depth"
        if points is None:
            return
        self.hub.set_cloud({"frame": frame, key: points})

    def _laser_points(self, frame):
        msg = self.scan
        if msg is None or not msg.ranges:
            return None
        xf = self._frame_xf(frame, msg.header.frame_id or "laser")
        if xf is None:
            return None
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float64) * msg.angle_increment
        keep = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < min(msg.range_max, 8.0))
        if not np.any(keep):
            return []
        dist = ranges[keep]
        ang = angles[keep]
        xs = dist * np.cos(ang)
        ys = dist * np.sin(ang)
        out_x, out_y, out_z = self._apply_xyz(xf, xs, ys, np.zeros_like(xs))
        return np.stack((np.round(out_x, 3), np.round(out_y, 3), np.round(out_z, 3)), axis=1).tolist()

    def _depth_points(self, frame):
        msg = self.depth
        if msg is None or msg.width == 0 or not msg.data:
            return None
        xf = self._frame_xf(frame, msg.header.frame_id or "base_footprint")
        if xf is None:
            return None
        x_off, y_off, z_off = 0, 4, 8
        for field in msg.fields:
            if field.name == "x":
                x_off = field.offset
            elif field.name == "y":
                y_off = field.offset
            elif field.name == "z":
                z_off = field.offset
        count = msg.width * msg.height
        step = msg.point_step
        if step < 12 or len(msg.data) < count * step:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < count * step:
            return None
        buf = raw[:count * step].reshape(count, step)
        xs = np.frombuffer(buf[:, x_off:x_off + 4].tobytes(), dtype=np.float32).astype(np.float64)
        ys = np.frombuffer(buf[:, y_off:y_off + 4].tobytes(), dtype=np.float32).astype(np.float64)
        zs = np.frombuffer(buf[:, z_off:z_off + 4].tobytes(), dtype=np.float32).astype(np.float64)
        keep = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        if not np.any(keep):
            return []
        out_x, out_y, out_z = self._apply_xyz(xf, xs[keep], ys[keep], zs[keep])
        return np.stack((np.round(out_x, 3), np.round(out_y, 3), np.round(out_z, 3)), axis=1).tolist()

    def _rgb(self, msg):
        if not self.publish_scene:
            return
        now = time.time()
        if now - self._rgb_time < 0.12:
            return
        self._rgb_time = now
        self._emit_rgb(msg)

    def _cloud_columns(self, msg):
        step = msg.point_step
        count = msg.width * msg.height
        if step <= 0 or count <= 0 or len(msg.data) < msg.row_step:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        width_bytes = msg.width * step
        if msg.row_step == width_bytes:
            if raw.size < count * step:
                return None
            return raw[:count * step].reshape(count, step)
        rows = []
        for row in range(msg.height):
            start = row * msg.row_step
            rows.append(raw[start:start + width_bytes])
        return np.concatenate(rows).reshape(count, step)

    def _emit_rgb(self, msg):
        frame = self._display_frame()
        if frame is None or msg.width == 0:
            return
        rt = self._frame_rt(frame, msg.header.frame_id or "camera_color_optical_frame")
        if rt is None:
            return
        cols = self._cloud_columns(msg)
        if cols is None:
            return
        offsets = {}
        types = {}
        for field in msg.fields:
            offsets[field.name] = field.offset
            types[field.name] = field.datatype
        if not all(name in offsets for name in ("x", "y", "z")):
            return
        color_name = "rgb" if "rgb" in offsets else ("rgba" if "rgba" in offsets else None)
        if color_name is None:
            rospy.logwarn_throttle(10.0, "彩色点云没有 rgb 字段")
            return

        def col(name):
            off = offsets[name]
            return np.frombuffer(cols[:, off:off + 4].tobytes(), dtype=np.float32)

        xs = col("x").astype(np.float64)
        ys = col("y").astype(np.float64)
        zs = col("z").astype(np.float64)
        keep = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        if not np.any(keep):
            self.hub.set_rgb({"frame": frame, "clear": False}, struct.pack("<I", 0))
            return
        bits = np.frombuffer(cols[:, offsets[color_name]:offsets[color_name] + 4].tobytes(), dtype=np.uint32)
        pts = np.stack((xs[keep], ys[keep], zs[keep]), axis=1)
        color = np.stack((
            ((bits[keep] >> 16) & 255) / 255.0,
            ((bits[keep] >> 8) & 255) / 255.0,
            (bits[keep] & 255) / 255.0,
        ), axis=1)
        if pts.shape[0] > 7000:
            pick = np.linspace(0, pts.shape[0] - 1, 7000).astype(np.int32)
            pts = pts[pick]
            color = color[pick]
        rotation, translation = rt
        world = pts.dot(rotation.T) + translation
        packed = np.concatenate((world.astype(np.float32), color.astype(np.float32)), axis=1)
        blob = struct.pack("<I", packed.shape[0]) + np.ascontiguousarray(packed).tobytes()
        self.hub.set_rgb({"frame": frame}, blob)

    def _frame_rt(self, target, source):
        if not target or not source:
            return None
        if target == source:
            return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)
        try:
            trans = self.tf_buffer.lookup_transform(target, source, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        quat = trans.transform.rotation
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)
        pos = trans.transform.translation
        return rotation, np.array([pos.x, pos.y, pos.z], dtype=np.float64)

    def _push_path(self, kind, msg):
        if not self.publish_scene:
            return
        frame = self._display_frame()
        if frame is None:
            return
        src = msg.header.frame_id or frame
        xf = self._frame_xf(frame, src)
        if xf is None:
            return
        points = []
        for pose in msg.poses:
            x, y = self._apply_xy(xf, pose.pose.position.x, pose.pose.position.y)
            points.append((round(float(x), 3), round(float(y), 3)))
        self.paths[kind] = points
        self.hub.set_paths({
            "frame": frame,
            "global": self.paths.get("global") or [],
            "local": self.paths.get("local") or [],
        })


class Handler(BaseHTTPRequestHandler):
    app = None

    def log_message(self, fmt, *args):
        if args and isinstance(args[0], str) and ("/events" in args[0] or "/camera.mjpg" in args[0] or "/map.png" in args[0] or "/costmap/" in args[0] or "/cloud/rgb" in args[0]):
            return
        rospy.loginfo(fmt, *args)

    def _send(self, code, body, content_type, extra=None):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        if extra:
            for key, value in extra.items():
                self.send_header(key, value)
        self.end_headers()
        self.wfile.write(data)
        self.app.hub.note(len(data))

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._file("index.html", "text/html; charset=utf-8")
        elif path == "/app.js":
            self._file("app.js", "text/javascript; charset=utf-8")
        elif path == "/view3d.js":
            self._file("view3d.js", "text/javascript; charset=utf-8")
        elif path == "/style.css":
            self._file("style.css", "text/css; charset=utf-8")
        elif path == "/events":
            self._events()
        elif path == "/camera.mjpg":
            self._camera()
        elif path == "/map.png":
            png = self.app.hub.map_png
            if not png:
                self._send(404, b"", "image/png")
                return
            self._send(200, png, "image/png")
        elif path in ("/costmap/global.png", "/costmap/local.png"):
            kind = "global" if "global" in path else "local"
            slot = self.app.hub.costs.get(kind) or {}
            png = slot.get("png") or b""
            if not png:
                self._send(404, b"", "image/png")
                return
            self._send(200, png, "image/png")
        elif path == "/cloud/rgb.bin":
            blob = self.app.hub.rgb_blob
            if not blob:
                self._send(404, b"", "application/octet-stream")
                return
            self._send(200, blob, "application/octet-stream")
        elif path == "/health":
            self._send(200, json.dumps(self.app.stack.snapshot()), "application/json")
        else:
            self._send(404, "not found", "text/plain")

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/api/cmd":
            self._send(404, "not found", "text/plain")
            return
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length > 100000:
            self._send(413, "too large", "text/plain")
            return
        try:
            body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self._send(400, "bad json", "text/plain")
            return
        op = body.get("op")
        if op == "teleop":
            self.app.bridge.set_cmd(body.get("linear", 0), body.get("angular", 0))
            self._send(200, '{"ok":true}', "application/json")
            return
        if op == "speeds":
            speeds = self.app.bridge.set_speeds(body)
            self._send(200, json.dumps({"ok": True, "speeds": speeds}), "application/json")
            return
        if op == "goal":
            try:
                self.app.bridge.stop_reloc_spin()
                self.app.bridge.publish_goal(body["x"], body["y"], body["yaw"])
            except (KeyError, TypeError, ValueError):
                self._send(400, '{"ok":false}', "application/json")
                return
            self._send(200, '{"ok":true}', "application/json")
            return
        if op == "reloc":
            try:
                self.app.bridge.publish_reloc(body["x"], body["y"], body["yaw"])
            except (KeyError, TypeError, ValueError):
                self._send(400, '{"ok":false}', "application/json")
                return
            self._send(200, '{"ok":true}', "application/json")
            return
        if op == "system":
            if body.get("confirm") is not True:
                self._send(400, json.dumps({"ok": False, "error": "需要确认"}, ensure_ascii=False), "application/json; charset=utf-8")
                return
            result = request_system_power(body.get("action"))
            self._send(200, json.dumps(result, ensure_ascii=False), "application/json; charset=utf-8")
            return
        result = self.app.stack.handle(op, body)
        self._send(200, json.dumps(result, ensure_ascii=False), "application/json; charset=utf-8")

    def _file(self, name, content_type):
        path = os.path.normpath(os.path.join(WEB, name))
        if not path.startswith(WEB) or not os.path.isfile(path):
            self._send(404, "missing", "text/plain")
            return
        with open(path, "rb") as handle:
            self._send(200, handle.read(), content_type)

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        seen = {"pose": -1, "state": -1, "map": -1, "cloud": -1, "paths": -1, "global": -1, "local": -1, "rgb": -1}
        hub = self.app.hub
        try:
            while not rospy.is_shutdown():
                with hub.cv:
                    hub.cv.wait(timeout=1.0)
                    pose_seq, state_seq, map_seq = hub.pose_seq, hub.state_seq, hub.map_seq
                    cloud_seq, paths_seq = hub.cloud_seq, hub.paths_seq
                    pose, state, meta = hub.pose, hub.state, hub.map_meta
                    cloud, paths = hub.cloud, hub.paths
                    rgb_seq, rgb_meta = hub.rgb_seq, hub.rgb_meta
                    costs = {key: slot.get("meta") for key, slot in hub.costs.items() if slot.get("meta")}
                    cost_seq = {key: slot.get("seq", 0) for key, slot in hub.costs.items()}
                sent = False
                if pose is not None and pose_seq != seen["pose"]:
                    sent = self._event("pose", pose) or sent
                    seen["pose"] = pose_seq
                if state and state_seq != seen["state"]:
                    sent = self._event("state", state) or sent
                    seen["state"] = state_seq
                if meta is not None and map_seq != seen["map"]:
                    sent = self._event("map", meta) or sent
                    seen["map"] = map_seq
                if cloud is not None and cloud_seq != seen["cloud"]:
                    sent = self._event("cloud", cloud) or sent
                    seen["cloud"] = cloud_seq
                if paths is not None and paths_seq != seen["paths"]:
                    sent = self._event("paths", paths) or sent
                    seen["paths"] = paths_seq
                if rgb_meta is not None and rgb_seq != seen["rgb"]:
                    sent = self._event("rgb", rgb_meta) or sent
                    seen["rgb"] = rgb_seq
                for kind in ("global", "local"):
                    if costs.get(kind) and cost_seq.get(kind) != seen[kind]:
                        sent = self._event("costmap", costs[kind]) or sent
                        seen[kind] = cost_seq.get(kind)
                if not sent:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _event(self, name, payload):
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        try:
            self.wfile.write(b"event: " + name.encode() + b"\ndata: " + data + b"\n\n")
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            raise
        self.app.hub.note(len(data) + len(name) + 16)
        return True

    def _camera(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = -1
        try:
            while not rospy.is_shutdown():
                blob, seq = self.app.hub.get_jpeg()
                if blob and seq != last:
                    part = (
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(blob)
                    ) + blob + b"\r\n"
                    self.wfile.write(part)
                    self.wfile.flush()
                    self.app.hub.note(len(part))
                    last = seq
                else:
                    time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


class App(object):
    def __init__(self):
        self.hub = Hub()
        self.stack = Stack(self.hub)
        self.bridge = Bridge(self.hub)
        self.stack.bridge = self.bridge
        self.stack.push()


def main():
    rospy.init_node("flyingshark_webui", anonymous=False)
    port = int(rospy.get_param("~port", 8080))
    app = App()
    Handler.app = app
    ThreadingHTTPServer.allow_reuse_address = True
    try:
        server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    except OSError as exc:
        rospy.logerr("端口 %d 无法监听: %s", port, exc)
        return
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    addresses = []
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True, timeout=2)
        addresses = [item for item in out.split() if item and not item.startswith("127.")]
    except Exception:
        addresses = []
    if not addresses:
        addresses = ["127.0.0.1"]
    for ip in addresses:
        line = "网页控制台 http://%s:%d/" % (ip, port)
        print(line, flush=True)
        rospy.loginfo(line)
    rospy.on_shutdown(server.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
