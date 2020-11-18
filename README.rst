rosutils
========

`rosutils` contains a bunch of python utility scripts:

.. code::

    addtime2topic.py

      Usage: addtime2topic.py <ros bag> <topic>
      Example: addtime2topic.py recorded.bag /robot/imu

      Often times one might forget to fill in the ros message header to include
      the timestamp. This script takes the rosbag and topic name as input and
      fills the timestamp field in each message according to the time it was
      written to the rosbag.


    bag2csv.py

      Usage: bag2csv.py <ros bag> <ros topic> <output path>
      Example: bag2csv.py record.bag /robot/pose robot_images.csv

      Converts a rosbag topic to csv file. Example, if the topic you want to
      convert to csv is a nav_msgs/Odometry msg, this script will parse and
      write each message to a csv file.


    bag2imgs.py

      Usage: bag2imgs.py <ros bag> <ros topic> <output path>
      Example: bag2imgs.py record.bag /robot/camera /tmp/images/

      Converts the rosbag topic to images.


    bag2vid.py

      Usage: bag2vid.py <ros bag> <image topic> <video output path>
      Example: bag2vid.py test.bag /camera/image video.avi

      Converts the rosbag topic to video.


    restamp_bag.py

      Usage: restamp_bag.py <input_file>
      Example: restamp_bag.py bursty_data.bag

      There are two types of timestamps in a rosbag. 1.) Time of which the
      message is wrtten to the bag, 2.) The message's own timestamp. There are
      scenarios where the time written to the rosbag is signficantly later than
      the timestamp in the message (i.e. delayed), this is common on high rate
      sensors such as an IMU where writing to bag in bursts is preferred.

      This script creates a restamped bag where the time writte to the rosbag
      is equal to the message timestamp itself.

    show_depth.py
      Script for visualizing depth sensor data.

    show_image.py
      Script for visualizing RGB sensor data.


LICENCE
-------

MIT
