#!/usr/bin/env python3
"""Publish a lighter copy of the RGB point cloud for RViz."""

import rospy
from sensor_msgs.msg import PointCloud2


class RgbCloudDownsample(object):
    def __init__(self):
        self.stride = max(1, int(rospy.get_param("~stride", 4)))
        self.pub = rospy.Publisher("rgb_points", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(
            "depth_registered/points", PointCloud2, self.callback, queue_size=1, buff_size=2**24
        )

    def callback(self, msg):
        stride = self.stride
        if msg.width < stride or msg.height < stride or msg.point_step <= 0:
            return
        width = msg.width // stride
        height = msg.height // stride
        step = msg.point_step
        out = PointCloud2()
        out.header = msg.header
        out.height = height
        out.width = width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.row_step = width * step
        out.is_dense = False
        raw = msg.data
        pieces = []
        for row in range(0, height * stride, stride):
            base = row * msg.row_step
            for col in range(0, width * stride, stride):
                start = base + col * step
                pieces.append(raw[start:start + step])
        out.data = b"".join(pieces)
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("rgb_cloud_downsample")
    RgbCloudDownsample()
    rospy.spin()
