#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import rosbag

from optparse import OptionParser
from datetime import datetime


def bag_to_csv(options, args):
    try:
        bag = rosbag.Bag(args[0])
    except Exception as e:
        rospy.logfatal('failed to load bag file: %s', e)
        exit(1)

    try:
        print('%time,')
        for topic, msg, time in bag.read_messages(topics=options.topic_names):
            print time, ','
    except Exception as e:
        rospy.logwarn("fail: %s", e)
    finally:
        bag.close()

if __name__ == '__main__':
    rospy.init_node('bag2csv', anonymous=True)
    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-t", "--topic", dest="topic_names",
                      action="append",
                      help="white list topic names", metavar="TOPIC_NAME")
    (options, args) = parser.parse_args()

    if len(args) != 1:
        parser.print_help()
        exit(0)

    bag_to_csv(options, args)
