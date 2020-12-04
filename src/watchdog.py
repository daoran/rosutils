#!/usr/bin/env python
import os
import sys

import rospy
from std_msgs.msg import Header

# GLOBAL VARIABLES
heartbeat_last = None


def print_usage():
    print("Usage: watchdog.py <rate in hz>")
    print("Example: watchdog.py 1")


def heartbeat_callback(msg):
    if heartbeat_last is None:
        heartbeat_last = msg.stamp
        return

    heartbeat_now = msg.stamp
    time_diff = heartbeat_now - heartbeat_last
    if time_diff > 1.0:
        rosp.logfatal("Time diff between heartbeat: %d [s]" % time_diff)
        exit(-1)

    heartbeat_last = heartbeat_now


if __name__ == '__main__':
    # Check CLI args
    if len(sys.argv) != 2:
        print_usage()
        exit(-1)

    # Parse CLI args
    rate = int(sys.argv[1])

    # Instanciate ROS node
    rospy.init_node('watchdog', anonymous=True)
    sub = rospy.Subscriber('/heartbeat', Header, heartbeat_callback)
    rate = rospy.Rate(rate)  # Hz

    # Listen to heartbeat
    rospy.spin()
