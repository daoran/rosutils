#!/usr/bin/env python
import os
import sys

import rospy
from std_msgs.msg import Header

def print_usage():
    print("Usage: heartbeat.py <rate in hz>")
    print("Example: heartbeat.py 1")

if __name__ == '__main__':
    # Check CLI args
    if len(sys.argv) != 2:
        print_usage()
        exit(-1)

    # Parse CLI args
    rate = int(sys.argv[1])

    # Instanciate ROS node
    rospy.init_node('heartbeat', anonymous=True)
    pub = rospy.Publisher('~/heartbeat', Header, queue_size=1)
    rate = rospy.Rate(rate)  # Hz

    # Start heartbeat
    seq = 0
    while not rospy.is_shutdown():
        msg = Header()
        msg.seq = seq
        msg.stamp = rospy.Time.now()
        msg.frame_id = "n/a"
        seq += 1

        pub.publish(msg)
        rate.sleep()
