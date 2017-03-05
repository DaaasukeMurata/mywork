#!/usr/bin/env python
# coding: UTF-8

import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf

import rospy
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from rc_image_w_tf.msg import ProcessedImage


# define

class Sender():

    def __init__(self):
        rospy.init_node('sender')
        self._cv_bridge = CvBridge()
        self._pub = rospy.Publisher('array_msg', UInt8MultiArray, queue_size=1)
        self._sub = rospy.Subscriber('/usb_cam_node/image_raw', Image, self.callback, queue_size=1)

    def callback(self, img_msg):
        img = self._cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        print img.shape

        out = np.reshape(img, -1)
        print out.shape

        snd_msg = ProcessedImage()
        snd_msg.height = img.shape[0]
        snd_msg.width = img.shape[1]
        snd_msg.depth = img.shape[2]        
        snd_msg.data = img.tolist()
        snd_msg.line_position = 200
        snd_msg.line_inclination = 0.5
        self._pub.publish(a)

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    process = Sender()
    process.main()
