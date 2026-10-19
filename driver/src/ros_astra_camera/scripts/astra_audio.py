#!/usr/bin/env python3
"""Publish the Astra Pro stereo microphone as left and right ROS topics.

The Orbbec camera node does not read this USB audio device. It is the
capture endpoint on the depth chip, fixed at 16 kHz, S16_LE, two channels
mapped as front-left and front-right.
"""

import subprocess

import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout


def split_stereo(raw):
    count = len(raw) // 4
    left = []
    right = []
    for index in range(count):
        base = index * 4
        left.append(int.from_bytes(raw[base:base + 2], "little", signed=True))
        right.append(int.from_bytes(raw[base + 2:base + 4], "little", signed=True))
    return left, right


def pack(samples):
    message = Int16MultiArray()
    message.layout = MultiArrayLayout(dim=[
        MultiArrayDimension(label="samples", size=len(samples), stride=len(samples))
    ], data_offset=0)
    message.data = samples
    return message


class AstraAudio(object):
    def __init__(self):
        self.device = rospy.get_param("~device", "hw:Pro,0")
        self.rate = int(rospy.get_param("~sample_rate", 16000))
        self.channels = int(rospy.get_param("~channels", 2))
        self.chunk = int(rospy.get_param("~chunk", 1600))
        self.left_pub = rospy.Publisher("audio/left", Int16MultiArray, queue_size=8)
        self.right_pub = rospy.Publisher("audio/right", Int16MultiArray, queue_size=8)
        self.frame_bytes = self.chunk * self.channels * 2

    def run(self):
        while not rospy.is_shutdown():
            proc = self.open_capture()
            if proc is None:
                rospy.sleep(1.0)
                continue
            rospy.loginfo("Astra 麦克风已打开：%s，%d Hz，左右声道", self.device, self.rate)
            try:
                self.read_loop(proc)
            finally:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
            if not rospy.is_shutdown():
                rospy.logwarn("Astra 麦克风中断，1 秒后重开")
                rospy.sleep(1.0)

    def open_capture(self):
        command = [
            "arecord", "-D", self.device, "-f", "S16_LE",
            "-r", str(self.rate), "-c", str(self.channels),
            "-t", "raw", "-q",
        ]
        try:
            return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as exc:
            rospy.logwarn("打不开 Astra 麦克风：%s", exc)
            return None

    def read_loop(self, proc):
        while not rospy.is_shutdown():
            raw = proc.stdout.read(self.frame_bytes)
            if len(raw) < self.frame_bytes:
                return
            left, right = split_stereo(raw)
            self.left_pub.publish(pack(left))
            self.right_pub.publish(pack(right))


if __name__ == "__main__":
    rospy.init_node("astra_audio")
    AstraAudio().run()
