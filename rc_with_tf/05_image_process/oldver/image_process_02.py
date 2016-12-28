# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np


class ImageShow():

    def __init__(self, rows, cols):
        self._rows = rows
        self._cols = cols
        self._cv_images = {}
        self._fig = plt.figure()

    def _reDraw(self, data):
        print ('ImageShow._reDraw() start')
        plt.cla()
        rand = np.random.randn(100)
        plt.plot(rand)
        # index = 1
        # for key in self._cv_images.keys():
        #     plt.subplot(self._rows, self._cols, index)
        #     plt.imshow(self._cv_images[key], 'gray')
        #     plt.title(key)
        #     index = index + 1

    def addCvImage(self, key, cv_image):
        # print ('addCvImage start')
        self._cv_images[key] = cv_image

    def show(self):
        print ('ImageShow.show() start')
        anim.FuncAnimation(self._fig, self._reDraw, interval=33)
        plt.show()


class ImageProcess():

    def __init__(self):
        self._show = ImageShow(2, 2)

        self._cv_bridge = CvBridge()
        self.cv_image_filterAdaptiveTh = None

        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)
        self._image_filterAdaptiveTh_pub = rospy.Publisher('image_filterAdaptiveTh', Image, queue_size=1)

    def rawTh(self, cv_image_gray):
        # raw ThresHolding
        ret, cv_image_processed = cv2.threshold(cv_image_gray, 128, 255, cv2.THRESH_BINARY_INV)
        self._image_rawTh_pub.publish(self._cv_bridge.cv2_to_imgmsg(cv_image_processed, 'mono8'))

    def adaptiveTh(self, cv_image_gray):
        # Adaptive Gaussian Thresholding
        cv_image_processed = cv2.adaptiveThreshold(cv_image_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                   cv2.THRESH_BINARY, 11, 2)
        self._image_adaptiveTh_pub.publish(self._cv_bridge.cv2_to_imgmsg(cv_image_processed, 'mono8'))

    def filterTh(self, cv_image_gray):
        # filter
        blur = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
        ret, cv_image_processed = cv2.threshold(blur, 70, 255, cv2.THRESH_BINARY_INV)
        self._image_filterTh_pub.publish(self._cv_bridge.cv2_to_imgmsg(cv_image_processed, 'mono8'))

    def filterAdaptiveTh(self, cv_image_gray):
        # filter + adaptive threshold
        blur = cv2.GaussianBlur(cv_image_gray, (5, 5), 0)
        self.cv_image_filterAdaptiveTh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                               cv2.THRESH_BINARY, 11, 2)
        self._image_filterAdaptiveTh_pub.publish(self._cv_bridge.cv2_to_imgmsg(self.cv_image_filterAdaptiveTh, 'mono8'))
        self._show.addCvImage('filterAdaptiveTh', self.cv_image_filterAdaptiveTh)

    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        rospy.loginfo('image process start')
        self.filterAdaptiveTh(cv_image_gray)

    def main(self):
        self._show.show()
        # rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_process')
    process = ImageProcess()
    process.main()
