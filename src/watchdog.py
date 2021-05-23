#!/usr/bin/env python
import os
import sys

import rospy
from std_msgs.msg import Header


def print_usage():
    print("Usage: watchdog.py")
    print("Example: watchdog.py")


def heartbeat_callback(msg):
    heartbeat_ts = msg.stamp
    now_ts = rospy.get_rostime()
    time_diff = now_ts - heartbeat_ts
    print("Time diff: %f [s]" % (time_diff.to_nsec() * 1e-9))


if __name__ == '__main__':
    # # Check CLI args
    # if len(sys.argv) != 2:
    #     print_usage()
    #     exit(-1)

    # # Parse CLI args
    # rate = int(sys.argv[1])

    # Instanciate ROS node
    rospy.init_node('watchdog', anonymous=True)
    sub = rospy.Subscriber('/heartbeat', Header, heartbeat_callback)
    # rate = rospy.Rate(rate)  # Hz

    # Listen to heartbeat
    rospy.spin()
