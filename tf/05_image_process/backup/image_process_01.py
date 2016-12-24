# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as anim

class ImageProcess():

    def __init__(self):
        self._cv_bridge = CvBridge()
        self.cv_image_filterAdaptiveTh = None

        self.fig = plt.figure()

        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)
        self._image28_pub = rospy.Publisher('image28', Image, queue_size=1)
        self._imagegray_pub = rospy.Publisher('imagegray', Image, queue_size=1)
        self._image_rawTh_pub = rospy.Publisher('image_rawTh', Image, queue_size=1)
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


    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        rospy.loginfo('image process start')
        self.filterAdaptiveTh(cv_image_gray)

        # cv_image_28 = cv2.resize(cv_image_binary, (28, 28))
        # np_image = np.reshape(cv_image_28, (1, 28, 28, 1))

    def plotAnim(self, data):
        plt.cla()
        # plt.imshow(self.cv_image_filterAdaptiveTh, 'gray', vmin = 0, vmax = 255) 
        plt.imshow(self.cv_image_filterAdaptiveTh, 'gray') 

    def main(self):
        ani = anim.FuncAnimation(self.fig, self.plotAnim, interval=33)
        plt.show()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_process')
    tensor = ImageProcess()
    tensor.main()
