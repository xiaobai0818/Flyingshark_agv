#!/usr/bin/env python3
"""Record surfaces above the lidar into the global occupancy map.

Cartographer still builds /map_slam from the laser only, so scan matching and
loop closure stay unchanged. This node remembers the depth points that sit
above the lidar and paints those cells occupied on /map. A later laser ray
under a bed does not erase them. Space behind the visible surface is not filled.
"""

import math
import threading

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def merge_hits(data, width, height, resolution, origin_x, origin_y, yaw, hits, hit_res):
    """Return grid, width, height, origin_x, origin_y with hit cells set occupied.

    hits are integer cells on a world grid of size hit_res. Cells the laser
    marked free stay occupied once a higher surface has been seen there.
    """
    width = int(width)
    height = int(height)
    resolution = float(resolution)
    if width <= 0 or height <= 0 or resolution <= 0:
        return None
    old = np.asarray(data, dtype=np.int16).reshape((height, width))
    if not hits:
        return old.reshape(-1).copy(), width, height, float(origin_x), float(origin_y)

    cells = np.asarray(list(hits), dtype=np.int32)
    xs = (cells[:, 0].astype(np.float64) + 0.5) * hit_res
    ys = (cells[:, 1].astype(np.float64) + 0.5) * hit_res
    if abs(yaw) < 1e-3:
        cols = np.floor((xs - origin_x) / resolution).astype(np.int32)
        rows = np.floor((ys - origin_y) / resolution).astype(np.int32)
        min_c = int(min(0, cols.min()))
        min_r = int(min(0, rows.min()))
        max_c = int(max(width - 1, cols.max()))
        max_r = int(max(height - 1, rows.max()))
        new_w = max_c - min_c + 1
        new_h = max_r - min_r + 1
        if min_c == 0 and min_r == 0 and new_w == width and new_h == height:
            grid = old.copy()
        else:
            grid = np.full((new_h, new_w), -1, dtype=np.int16)
            grid[-min_r:-min_r + height, -min_c:-min_c + width] = old
            origin_x = float(origin_x) + min_c * resolution
            origin_y = float(origin_y) + min_r * resolution
            cols = cols - min_c
            rows = rows - min_r
            width, height = new_w, new_h
        inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        grid[rows[inside], cols[inside]] = 100
        return grid.reshape(-1).copy(), width, height, float(origin_x), float(origin_y)

    c = math.cos(yaw)
    s = math.sin(yaw)
    dx = xs - origin_x
    dy = ys - origin_y
    cols = np.floor((c * dx + s * dy) / resolution).astype(np.int32)
    rows = np.floor((-s * dx + c * dy) / resolution).astype(np.int32)
    grid = old.copy()
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    grid[rows[inside], cols[inside]] = 100
    return grid.reshape(-1).copy(), width, height, float(origin_x), float(origin_y)


class HeightMapOverlay(object):
    def __init__(self):
        self.min_z = float(rospy.get_param("~min_z", 0.12))
        self.max_z = float(rospy.get_param("~max_z", 0.65))
        self.hit_res = float(rospy.get_param("~resolution", 0.05))
        self.hits = set()
        self.lock = threading.Lock()
        self.slam = None
        self.dirty = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("map", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber("map_slam", OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber("depth_obstacles", PointCloud2, self.on_cloud, queue_size=1, buff_size=2 ** 20)
        rospy.Timer(rospy.Duration(0.5), self.on_timer)

    def on_cloud(self, msg):
        try:
            pts = np.array(
                list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float64,
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "高处地图: 点云无法读取 (%s)", exc)
            return
        if pts.ndim != 2 or pts.shape[0] == 0:
            return
        keep = (pts[:, 2] >= self.min_z) & (pts[:, 2] <= self.max_z)
        pts = pts[keep]
        if pts.size == 0:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1)
            )
        except tf2_ros.TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.05)
                )
            except tf2_ros.TransformException as exc:
                rospy.logwarn_throttle(5.0, "高处地图: 还没有 map 坐标 (%s)", exc)
                return
        rot = rotation_matrix(transform.transform.rotation)
        trans = transform.transform.translation
        world = pts.dot(rot.T)
        world[:, 0] += trans.x
        world[:, 1] += trans.y
        world[:, 2] += trans.z
        keys = np.floor(world[:, :2] / self.hit_res).astype(np.int32)
        added = 0
        with self.lock:
            before = len(self.hits)
            self.hits.update((int(ix), int(iy)) for ix, iy in keys)
            added = len(self.hits) - before
            if added:
                self.dirty = True
        if added:
            rospy.loginfo_throttle(2.0, "高处地图: 新记入 %d 格，累计 %d 格", added, len(self.hits))

    def on_map(self, msg):
        with self.lock:
            self.slam = msg
            self.dirty = True
        self.publish()

    def on_timer(self, _event):
        with self.lock:
            dirty = self.dirty
        if dirty:
            self.publish()

    def publish(self):
        with self.lock:
            slam = self.slam
            hits = set(self.hits)
            self.dirty = False
        if slam is None or slam.info.width <= 0 or slam.info.height <= 0:
            return
        info = slam.info
        merged = merge_hits(
            slam.data,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            yaw_from_quat(info.origin.orientation),
            hits,
            self.hit_res,
        )
        if merged is None:
            return
        data, width, height, origin_x, origin_y = merged
        out = OccupancyGrid()
        out.header.stamp = slam.header.stamp
        out.header.frame_id = slam.header.frame_id or "map"
        out.info.resolution = info.resolution
        out.info.width = int(width)
        out.info.height = int(height)
        out.info.origin.position.x = origin_x
        out.info.origin.position.y = origin_y
        out.info.origin.position.z = info.origin.position.z
        out.info.origin.orientation = info.origin.orientation
        out.data = data.astype(np.int8).tolist()
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("height_map_overlay")
    HeightMapOverlay()
    rospy.spin()
