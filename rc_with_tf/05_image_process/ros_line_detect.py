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

        # 処理負荷軽減のための事前縮小
        pre_scale = 1.0 / ParamServer.get_value('system.pre_resize')
        pimg.resize(pre_scale)

        # 抽象化
        pimg.preprocess()

        # 直線検出
        if ParamServer.get_value('system.detect_line'):
            pre_img = pimg.get_img()
            pimg.detect_line()
            pimg.overlay(pre_img)

        # deep learning学習データ用の縮小。 pre_resize * final_resizeの値が最終データとなる
        final_scale = 1.0 / ParamServer.get_value('system.final_resize')
        pimg.resize(final_scale)

        if ParamServer.get_value('system.mono_output'):
            self.__pub.publish(self.__cv_bridge.cv2_to_imgmsg(pimg.get_grayimg(), 'mono8'))
        else:
            self.__pub.publish(self.__cv_bridge.cv2_to_imgmsg(pimg.get_img(), 'bgr8'))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    gui_mode = True
    log_mode = False
    for arg in sys.argv:
        if (arg == '--disable-gui'):
            gui_mode = False
        elif (arg == '--logmode'):
            log_mode = True

    if (log_mode):
        ParamServer.set_value('system.detect_line', 0)
        ParamServer.set_value('system.mono_output', 1)

    process = RosLineDetect()

    if (gui_mode):
        app = QApplication(sys.argv)
        gui = setting_gui.SettingWindow()
        app.exec_()
    else:
        process.main()
