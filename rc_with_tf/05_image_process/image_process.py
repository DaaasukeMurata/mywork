# -*- coding: utf-8 -*-

import cv2
import numpy as np
from param_server import ParamServer


class ProcessingImage():

    def __init__(self, img):
        self.img = img

    # 現在grayでも3channel colorで返す。
    def getimg(self):
        if len(self.img.shape) < 3:     # iplimage.shape is [x,y,colorchannel]
            return cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        else:
            return self.img

    def __to_gray(self):
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

    def __detect_edge(self):
        EDGE_TH_LOW = ParamServer.get_param('canny.th_low')
        EDGE_TH_HIGH = ParamServer.get_param('canny.th_high')
        self.img = cv2.Canny(self.img, EDGE_TH_LOW, EDGE_TH_HIGH)

    def __threshold(self):
        self.img = cv2.adaptiveThreshold(self.img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)

    def __blur(self):
        FILTER_SIZE = (ParamServer.get_param('gaublue.filter_size'), ParamServer.get_param('gaublue.filter_size'))
        # bilateralFilterだと色の差も加味してそう
        # self.img = cv2.bilateralFilter(self.img, 5, 75, 75)
        self.img = cv2.GaussianBlur(self.img, FILTER_SIZE, 0)

    def __mask(self, vertices):
        # defining a blank mask to start with
        mask = np.zeros_like(self.img)

        # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(self.img.shape) > 2:
            channel_count = self.img.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        vertices[0][0:, 0] = vertices[0][0:, 0] * self.img.shape[1]
        vertices[0][0:, 1] = vertices[0][0:, 1] * self.img.shape[0]

        int_vertices = vertices.astype(np.int32)

        # filling pixels inside the polygon defined by "vertices" with the fill color
        cv2.fillPoly(mask, int_vertices, ignore_mask_color)

        # trancerate the image only where mask pixels are nonzero
        self.img = cv2.bitwise_and(self.img, mask)

    def __color_filter(self):
        LOW_B = ParamServer.get_param('color.low_b')
        LOW_G = ParamServer.get_param('color.low_g')
        LOW_R = ParamServer.get_param('color.low_r')
        HIGH_B = ParamServer.get_param('color.high_b')
        HIGH_G = ParamServer.get_param('color.high_g')
        HIGH_R = ParamServer.get_param('color.high_r')

        lower = np.array([LOW_B, LOW_G, LOW_R])
        upper = np.array([HIGH_B, HIGH_G, HIGH_R])

        hsv_image = cv2.cvtColor(self.getimg(), cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        self.img = cv2.bitwise_and(self.getimg(), self.getimg(), mask=mask_image)
        area = cv2.countNonZero(mask_image)
        return area

    def __houghline(self):
        THRESHOLD = ParamServer.get_param('houghline.threshold')
        MIN_LINE_LENGTH = ParamServer.get_param('houghline.min_line_length')
        MAX_LINE_GAP = ParamServer.get_param('houghline.max_line_gap')
        return cv2.HoughLinesP(self.img, 1, np.pi / 180, THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP)

    def __get_segment(self, x1, y1, x2, y2):
        vy = y2 - y1
        vx = x2 - x1

        if vx == 0:
            m = 0.
        else:
            m = (float)(vy) / vx

        b = y1 - (m * x1)
        return m, b

    def __get_point_horizontal(self, m, b, y_ref):
        x = (y_ref - b) / m
        return x

    def __extrapolation_lines(self, lines):
        # 検出する線の傾き範囲
        EXPECT_RIGHT_LINE_M_MIN = ParamServer.get_param('extrapolation_lines.right_m_min')
        EXPECT_RIGHT_LINE_M_MAX = ParamServer.get_param('extrapolation_lines.right_m_max')
        EXPECT_LEFT_LINE_M_MIN = ParamServer.get_param('extrapolation_lines.left_m_min')
        EXPECT_LEFT_LINE_M_MAX = ParamServer.get_param('extrapolation_lines.left_m_max')

        if lines is None:
            return None

        right_line = np.empty((0, 6), float)
        left_line = np.empty((0, 6), float)

        for line in lines:
            for tx1, ty1, tx2, ty2 in line:
                if ty2 > ty1:
                    x1 = tx1
                    x2 = tx2
                    y1 = ty1
                    y2 = ty2
                else:
                    x1 = tx2
                    x2 = tx1
                    y1 = ty2
                    y2 = ty1

                m, b = self.__get_segment(x1, y1, x2, y2)
                if EXPECT_RIGHT_LINE_M_MIN < m < EXPECT_RIGHT_LINE_M_MAX:
                    # right side
                    right_line = np.append(right_line, np.array([[x1, y1, x2, y2, m, b]]), axis=0)
                elif EXPECT_LEFT_LINE_M_MIN < m < EXPECT_LEFT_LINE_M_MAX:
                    # left side
                    left_line = np.append(left_line, np.array([[x1, y1, x2, y2, m, b]]), axis=0)

        # print 'right lines num:', right_line.size
        # print 'left lines num:', left_line.size

        if (right_line.size == 0) and (left_line.size == 0):
            return None

        extrapolation_lines = []

        if (right_line.size > 0):

            right_m = right_line[:, 4].mean(axis=0)
            right_b = right_line[:, 5].mean(axis=0)
            right_y_max = right_line[:, 3].max(axis=0)
            right_y_min = right_line[:, 1].min(axis=0)

            right_x_min = self.__get_point_horizontal(right_m, right_b, right_y_min)
            right_x_max = self.__get_point_horizontal(right_m, right_b, right_y_max)

            right_x_min = int(right_x_min)
            right_x_max = int(right_x_max)
            right_y_min = int(right_y_min)
            right_y_max = int(right_y_max)

            extrapolation_lines.append([[right_x_min, right_y_min, right_x_max, right_y_max]])

        if (left_line.size > 0):

            left_m = left_line[:, 4].mean(axis=0)
            left_b = left_line[:, 5].mean(axis=0)
            left_y_max = left_line[:, 3].max(axis=0)
            left_y_min = left_line[:, 1].min(axis=0)

            left_x_min = self.__get_point_horizontal(left_m, left_b, left_y_min)
            left_x_max = self.__get_point_horizontal(left_m, left_b, left_y_max)

            left_x_min = int(left_x_min)
            left_x_max = int(left_x_max)
            left_y_min = int(left_y_min)
            left_y_max = int(left_y_max)

            extrapolation_lines.append([[left_x_min, left_y_min, left_x_max, left_y_max]])

        return extrapolation_lines

    def preprocess(self):
        self.__color_filter()
        self.__to_gray()
        self.__blur()
        self.__detect_edge()

    def detect_line(self, color_pre=[0, 255, 0], color_final=[0, 0, 255], thickness=4):
        MASK_V1 = [0. / 640., 400. / 480.]
        MASK_V2 = [0. / 640., 200. / 480.]
        MASK_V3 = [640. / 640., 200. / 480.]
        MASK_V4 = [640. / 640., 400. / 480.]

        # image mask
        vertices = np.array([[MASK_V1, MASK_V2, MASK_V3, MASK_V4]], dtype=np.float)
        self.__mask(vertices)

        # line detect
        pre_lines = self.__houghline()
        final_lines = self.__extrapolation_lines(pre_lines)

        # create image
        if len(self.img.shape) == 3:
            line_img = np.zeros((self.img.shape), np.uint8)
        else:
            line_img = np.zeros((self.img.shape[0], self.img.shape[1], 3), np.uint8)

        # draw pre_lines
        if (pre_lines is None):
            return
        for x1, y1, x2, y2 in pre_lines[0]:
            cv2.line(line_img, (x1, y1), (x2, y2), color_pre, thickness)
        self.img = line_img

        # draw final_lines
        if (final_lines is None):
            return
        for x1, y1, x2, y2 in final_lines[0]:
            cv2.line(line_img, (x1, y1), (x2, y2), color_final, thickness)
        self.img = line_img

    def overlay(self, img):
        ALPHA = 1.0
        BETA = 0.8
        GAMMA = 1.8
        color_img = self.getimg()
        self.img = cv2.addWeighted(color_img, ALPHA, img, BETA, GAMMA)
