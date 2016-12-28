# -*- coding: utf-8 -*-

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np



class RosBagCreator():

    def __init__(self):
        rospy.init_node('rosbag_creator')
        self.__cv_bridge = CvBridge()
        self.__sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)
        self.__pub = rospy.Publisher('image_processed', Image, queue_size=1)


if __name__ == '__main__':
    process = RosBagCreator()





