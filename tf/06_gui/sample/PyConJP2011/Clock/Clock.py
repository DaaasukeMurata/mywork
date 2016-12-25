# -*- coding: utf-8 -*-
import sys
import math
import datetime
import PyQt4.QtCore as QtCore
import PyQt4.QtGui as QtGui


class ClockWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent=parent)
        # 表示領域の大きさ
        self.width = 320
        self.height = 320

        # 目盛りの色とサイズ
        self.large_scale_color = QtCore.Qt.white
        self.large_scale_length = 10
        self.large_scale_width = 3
        
        self.small_scale_color = QtCore.Qt.gray
        self.small_scale_length = 5
        self.small_scale_width = 1

        # 針の色とサイズ
        self.sec_needle_pen = QtGui.QPen(QtCore.Qt.yellow, 2)
        self.sec_needle_length = 50
        self.sec_needle_width_deg = 5

        self.min_needle_pen = QtGui.QPen(QtCore.Qt.green, 2)
        self.min_needle_length= 40
        self.min_needle_width_deg= 10
        
        self.hour_needle_pen = QtGui.QPen(QtCore.Qt.blue, 3)
        self.hour_needle_length = 20
        self.hour_needle_width_deg = 15
        
                
    def polar_to_plain(self, deg, r):
        "極座標から平面座標への変換"
        center_x = self.width / 2
        center_y = self.height / 2

        radian = deg / 180 * math.pi

        return (center_x + (math.sin(radian) * r),
                center_y - (math.cos(radian) * r))


    def calc_needle_coordinates(self, deg, length, width_deg):
        "針を表現する三角形の座標値を計算してQPolygonインスタンスを返す"
        outer_r = min(self.width, self.height) / 2 - 5 - self.large_scale_length
        deg_delta = width_deg / 2.0

        polygon = QtGui.QPolygon(4)

        polygon.setPoint(0, *self.polar_to_plain(deg, outer_r))
        polygon.setPoint(1, *self.polar_to_plain(deg + deg_delta, outer_r - length))
        polygon.setPoint(2, *self.polar_to_plain(deg - deg_delta, outer_r - length))
        polygon.setPoint(3, *self.polar_to_plain(deg, outer_r))

        return polygon
    

    def draw_needles(self):
        "現在時刻に合わせて針を描画する"
        painter = QtGui.QPainter()
        painter.begin(self.offscreen)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        now = datetime.datetime.now()

        # 時針の描画
        deg = (now.hour % 12) * 30 + (now.minute / 2.0) + (now.second / 120.0)
        painter.setPen(self.hour_needle_pen)
        painter.drawPolyline(
            self.calc_needle_coordinates(
                deg, self.hour_needle_length, self.hour_needle_width_deg))
        
        # 分針の描画
        deg = (now.minute * 6) + (now.second / 10.0)
        painter.setPen(self.min_needle_pen)
        painter.drawPolyline(
            self.calc_needle_coordinates(
                deg, self.min_needle_length, self.min_needle_width_deg))
        
        # 秒針の描画
        deg = (now.second + (now.microsecond / 1000) / 1000.0) * 6
        painter.setPen(self.sec_needle_pen)
        painter.drawPolyline(
            self.calc_needle_coordinates(
                deg, self.sec_needle_length, self.sec_needle_width_deg))
        
        painter.end()


    def draw_scales(self):
        "目盛りの描画"
        self.offscreen.fill(QtCore.Qt.black)
        
        large_pen = QtGui.QPen(self.large_scale_color, self.large_scale_width)
        small_pen = QtGui.QPen(self.small_scale_color, self.small_scale_width)

        painter = QtGui.QPainter()
        painter.begin(self.offscreen)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        outer_r = min(self.width, self.height) / 2 - 5
        
        for i in range(0, 60):
            (outer_x, outer_y) = self.polar_to_plain(i * 6, outer_r)
            
            if (i % 5) == 0:
                painter.setPen(large_pen)
                (inner_x, inner_y) = self.polar_to_plain(
                    i * 6, outer_r - self.large_scale_length)
            else:
                painter.setPen(small_pen)
                (inner_x, inner_y) = self.polar_to_plain(
                    i * 6, outer_r - self.small_scale_length)

            painter.drawLine(outer_x, outer_y, inner_x, inner_y)

        painter.end()


    def redraw_clock(self):
        "時計全体を再描画"
        self.draw_scales()
        self.draw_needles()
        self.update()

        
    def setup_ui(self):
        "コンポーネントの生成とタイマーの開始"
        self.setFixedSize(self.width, self.height)
        self.offscreen = QtGui.QPixmap(self.width, self.height)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.connect(self.timer, QtCore.SIGNAL("timeout()"), self.redraw_clock)
        self.timer.start()


    def paintEvent(self, paint_event):
        "再描画イベントに応答するメソッド"
        painter = QtGui.QPainter()
        painter.begin(self)
        painter.drawPixmap(0, 0, self.offscreen)
        painter.end()

        

def main():
    app = QtGui.QApplication(sys.argv)
    
    clock_widget = ClockWidget()
    clock_widget.setup_ui()
    
    main_window = QtGui.QMainWindow()
    main_window.setWindowTitle("Clock")
    main_window.setCentralWidget(clock_widget)
    main_window.show()

    app.exec_()


if __name__ == "__main__":
    main()
