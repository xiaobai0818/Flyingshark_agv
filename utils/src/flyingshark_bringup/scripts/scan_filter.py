#!/usr/bin/env python3
"""Drop laser returns on the robot body and optionally crop an angle sector."""

import math

import rospy
from sensor_msgs.msg import LaserScan


def normalize(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ScanFilter(object):
    def __init__(self):
        self.min_range = rospy.get_param("~min_range", 0.12)
        self.max_range = rospy.get_param("~max_range", 8.0)
        self.crop = rospy.get_param("~enable_angle_crop", False)
        self.crop_min = math.radians(rospy.get_param("~angle_crop_min_deg", 135.0))
        self.crop_max = math.radians(rospy.get_param("~angle_crop_max_deg", 225.0))
        self.pub = rospy.Publisher("scan", LaserScan, queue_size=5)
        self.sub = rospy.Subscriber("scan_raw", LaserScan, self.callback, queue_size=5)

    def in_crop(self, angle):
        angle = normalize(angle)
        start = normalize(self.crop_min)
        end = normalize(self.crop_max)
        if start <= end:
            return start <= angle <= end
        return angle >= start or angle <= end

    def callback(self, msg):
        ranges = list(msg.ranges)
        angle = msg.angle_min
        for i, distance in enumerate(ranges):
            masked = self.crop and self.in_crop(angle)
            if masked or (not math.isnan(distance) and distance < self.min_range):
                ranges[i] = float("nan")
            elif not math.isnan(distance) and distance > self.max_range:
                ranges[i] = float("inf")
            angle += msg.angle_increment
        msg.ranges = ranges
        msg.range_min = self.min_range
        msg.range_max = self.max_range
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("scan_filter")
    ScanFilter()
    rospy.spin()
