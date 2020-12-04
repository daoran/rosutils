#!/usr/bin/env python
import sys

import rospy
import rosbag


def print_usage():
    print('Usage: ts2csv.py <input_file> <topic> <save_path>')
    print('Example: ts2csv.py bursty_data.bag /some_topic ./timestamps.csv')


def check_topic_exists(bag, topic):
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic not in bag!")


if __name__ == "__main__":
    # Parse command-line args
    if len(sys.argv) != 4:
	print_usage()
	exit(-1)

    bag_path = sys.argv[1]
    topic = sys.argv[2]
    save_path = sys.argv[3]
    bag = rosbag.Bag(bag_path, 'r')

    # Checks
    check_topic_exists(bag, topic)

    # Setup output file
    csv_file = open(save_path, "w")
    csv_file.write("#secs,nsecs\n")

    # Convert timestamps to csv
    msg_counter = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Message timestamps
        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs

        # Recorded timestamp (a.k.a. time it was written into rosbag)
        # secs = t.secs
        # nsecs = t.nsecs

        csv_file.write("%d,%d\n" % (secs, nsecs))
        if (msg_counter % 100) == 0:
            sys.stdout.write('.')
            sys.stdout.flush()
        msg_counter += 1

    print("")
    print("Results saved to [%s]" % save_path)
    csv_file.close()
