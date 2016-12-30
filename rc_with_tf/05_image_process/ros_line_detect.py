# -*- coding: utf-8 -*-

# [how to use]
# python ros_line_detect.py image:=/image_raw


import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import image_process
import setting_gui
from param_server import ParamServer


class RosLineDetect():

    def __init__(self):
        rospy.init_node('image_process')
        self.__cv_bridge = CvBridge()
        self.__sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)
        self.__pub = rospy.Publisher('image_processed', Image, queue_size=1)
        ParamServer.add_cb_value_changed(self.redraw)

    def redraw(self):
        self.callback(self.last_image_msg)

    def callback(self, image_msg):
        self.last_image_msg = image_msg
        cv_image = self.__cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        pimg = image_process.ProcessingImage(cv_image)
        pimg.preprocess()
        pre_img = pimg.getimg()
        pimg.detect_line()
        pimg.overlay(pre_img)

        self.__pub.publish(self.__cv_bridge.cv2_to_imgmsg(pimg.getimg(), 'bgr8'))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    process = RosLineDetect()

    if (len(sys.argv) > 2) and (sys.argv[1] == '--disable-gui'):
        process.main()
    else:
        app = QApplication(sys.argv)
        gui = setting_gui.SettingWindow()
        app.exec_()
