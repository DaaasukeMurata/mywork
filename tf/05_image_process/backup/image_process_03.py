# -*- coding: utf-8 -*-

# [how to use]
# python image_process.py image:=/image_raw

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from collections import OrderedDict


class ImageShow():

    def __init__(self, rows, cols):
        self._rows = rows      # for matplotlib
        self._cols = cols      # for matplotlib
        self._max_num = rows * cols
        self._cv_images = OrderedDict()     # support GrayScale only
        self._fig = plt.figure()

    def _reDraw(self, data):
        # print ('ImageShow._reDraw() start')
        plt.cla()
        index = 0
        for key in self._cv_images.keys():
            index = index + 1
            if (index > self._max_num):
                ros.loginfo('show space is overflow')
                break
            plt.subplot(self._rows, self._cols, index)
            plt.imshow(self._cv_images[key], 'gray')
            plt.title(key)
            plt.tick_params(labelbottom='off', labelleft='off')
            plt.subplots_adjust(left=0.075, bottom=0.05, right=0.95, top=0.95, wspace=0.15, hspace=0.15)

    def addCvImage(self, key, cv_image):
        self._cv_images[key] = cv_image

    def show(self):
        ani = anim.FuncAnimation(self._fig, self._reDraw, interval=300)
        plt.show()


class ImageProcess():

    def __init__(self):
        self._show = ImageShow(2, 3)
        self._cv_bridge = CvBridge()
        self._cv_image_filterAdaptiveTh = None
        rospy.init_node('image_process')
        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)

    # Raw Th
    def _rawTh(self, cv_image_gray):
        ret, cv_image_th = cv2.threshold(cv_image_gray, 128, 255, cv2.THRESH_BINARY_INV)
        return cv_image_th

    # Adaptive Gaussian Th
    def _adaptiveTh(self, cv_image_gray):
        cv_image_th = cv2.adaptiveThreshold(cv_image_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY, 11, 2)
        return cv_image_th

    # Filter Th
    def _filterTh(self, cv_image_gray):
        blur = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
        ret, cv_image_th = cv2.threshold(blur, 70, 255, cv2.THRESH_BINARY_INV)
        return cv_image_th

    # Filter + AdaptiveGaussian Th
    def _filterAdaptiveTh(self, cv_image_gray):
        blur = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
        cv_image_th = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY, 11, 2)
        return cv_image_th

    def callback(self, image_msg):
        # rospy.loginfo('image process start')
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        self._show.addCvImage('Original(gray)', cv_gray)
        self._show.addCvImage('Raw Th', self._rawTh(cv_gray))
        self._show.addCvImage('Adaptive Gaussian Th', self._adaptiveTh(cv_gray))
        self._show.addCvImage('Filter Th', self._filterTh(cv_gray))
        self._show.addCvImage('Filter + AdaptiveGaussian Th', self._filterAdaptiveTh(cv_gray))

    def main(self):
        self._show.show()
        # rospy.spin()


if __name__ == '__main__':
    process = ImageProcess()
    process.main()
