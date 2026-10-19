#!/usr/bin/env python3
"""Keep the depth points the lidar cannot see, and nothing behind them.

Points between the lidar plane and a bed or table edge become obstacles.
Space past that surface stays free, so a goal behind the furniture can still
be reached by going around.
"""

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, read_points
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def downsample(msg, stride):
    if msg.point_step <= 0 or msg.width < stride:
        return None
    step = msg.point_step
    raw = msg.data
    out = PointCloud2()
    out.header = msg.header
    out.fields = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = step
    out.is_dense = False
    pieces = bytearray()
    if msg.height >= stride and msg.row_step >= msg.width * step:
        height = msg.height // stride
        width = msg.width // stride
        for row in range(0, height * stride, stride):
            base = row * msg.row_step
            for col in range(0, width * stride, stride):
                start = base + col * step
                pieces += raw[start:start + step]
        out.height = height
        out.width = width
    else:
        count = msg.width // stride
        for index in range(0, count * stride, stride):
            start = index * step
            pieces += raw[start:start + step]
        out.height = 1
        out.width = count
    if out.width == 0:
        return None
    out.row_step = out.width * step
    out.data = bytes(pieces)
    return out


def visible_surface(points, min_z, max_z, cell):
    """One point per grid cell on the surface that was actually seen."""
    if points.size == 0:
        return []
    zok = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    pts = points[zok]
    if pts.size == 0:
        return []
    rng = np.hypot(pts[:, 0], pts[:, 1])
    pts = pts[(rng > 0.25) & (rng < 4.5)]
    if pts.size == 0:
        return []
    keys = np.floor(pts[:, :2] / cell).astype(np.int32)
    _uniq, index = np.unique(keys, axis=0, return_index=True)
    return [tuple(p) for p in pts[index]]


class DepthObstacles(object):
    def __init__(self):
        self.stride = max(1, int(rospy.get_param("~stride", 4)))
        self.min_z = float(rospy.get_param("~min_z", 0.12))
        self.max_z = float(rospy.get_param("~max_z", 0.65))
        self.cell = float(rospy.get_param("~cell", 0.05))
        self.frame = rospy.get_param("~target_frame", "base_footprint")
        self.min_period = 1.0 / float(rospy.get_param("~rate", 5.0))
        self.last = rospy.Time(0)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("depth_obstacles", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(
            "depth/points", PointCloud2, self.callback, queue_size=1, buff_size=2**24
        )

    def callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last).to_sec() < self.min_period:
            return
        small = downsample(msg, self.stride)
        if small is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame, small.header.frame_id, small.header.stamp, rospy.Duration(0.15)
            )
        except tf2_ros.TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame, small.header.frame_id, rospy.Time(0), rospy.Duration(0.05)
                )
            except tf2_ros.TransformException as exc:
                rospy.logwarn_throttle(5.0, "depth obstacles: no tf to %s (%s)", self.frame, exc)
                return
        posed = do_transform_cloud(small, transform)
        pts = np.array(list(read_points(posed, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] == 0:
            return
        surface = visible_surface(pts, self.min_z, self.max_z, self.cell)
        if not surface:
            return
        header = posed.header
        header.stamp = msg.header.stamp
        header.frame_id = self.frame
        self.pub.publish(create_cloud_xyz32(header, surface))
        self.last = now


if __name__ == "__main__":
    rospy.init_node("depth_cloud_downsample")
    DepthObstacles()
    rospy.spin()
