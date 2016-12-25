# -*- coding: utf-8 -*-

from PyQt4.QtGui import *
from PyQt4.QtCore import *
import sys


class SliderSetting(QWidget):

    def __init__(self, name, range_low, range_high, init_value, parent=None ):
        QWidget.__init__(self, parent=parent)
        self.setup_ui(name, range_low, range_high, init_value)

    def setup_ui(self, name, range_low, range_high, init_value):
        slider_label = QLabel(name)
        self.slider = QSlider(Qt.Horizontal)  # スライダの向き
        self.slider.setRange(range_low, range_high)  # スライダの範囲
        self.slider.setValue(init_value)  # 初期値
        self.slider.setTickPosition(QSlider.TicksAbove)  # スライダのメモリの位置
        self.connect(self.slider, SIGNAL('valueChanged(int)'), self.on_draw)
        self.textbox = QLineEdit()

        layout = QHBoxLayout()
        layout.addWidget(slider_label, 2)
        layout.addWidget(self.slider, 5)
        layout.addWidget(self.textbox, 1)
        self.setLayout(layout)

    def on_draw(self):
        self.textbox.setText(str(self.slider.value()))


class App(QMainWindow):

    def main(self):
        self.w = QWidget()
        self.w.resize(500, 150)
        self.w.setWindowTitle('Parameter')

        slider_widget1 = SliderSetting('Slider:', 0, 100, 20)
        slider_widget2 = SliderSetting('settings sl:', 10, 20, 15)
        slider_widget3 = SliderSetting('settings sl:', 10, 20, 15)
        slider_widget4 = SliderSetting('settings sl:', 10, 20, 15)

        w_layout = QVBoxLayout()
        w_layout.addWidget(slider_widget1)
        w_layout.addWidget(slider_widget2)
        w_layout.addWidget(slider_widget3)
        w_layout.addWidget(slider_widget4)
        self.w.setLayout(w_layout)

        self.w.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainApp = App()
    mainApp.main()
    app.exec_()
