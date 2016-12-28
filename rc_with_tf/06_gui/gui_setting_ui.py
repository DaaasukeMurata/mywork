# -*- coding: utf-8 -*-

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from collections import OrderedDict
import sys


class SliderSetting(QWidget):

    def __init__(self, worker, name, init_value, range_low, range_high, parent=None):
        QWidget.__init__(self, parent=parent)
        self.worker = worker
        self.setup_ui(name, init_value, range_low, range_high)

    def setup_ui(self, name, init_value, range_low, range_high):
        self.dpi = 1
        if isinstance(init_value, float):
            self.dpi = 10

        self.slider_label = QLabel(name)
        self.slider = QSlider(Qt.Horizontal)  # スライダの向き
        self.slider.setRange(range_low * self.dpi, range_high * self.dpi)  # スライダの範囲
        self.slider.setValue(init_value * self.dpi)
        self.slider.setTickPosition(QSlider.TicksAbove)  # メモリの位置
        self.connect(self.slider, SIGNAL('valueChanged(int)'), self.on_draw)
        self.textbox = QLineEdit()
        self.textbox.setText(str(init_value))

        layout = QHBoxLayout()
        layout.addWidget(self.slider_label, 4)
        layout.addWidget(self.slider, 5)
        layout.addWidget(self.textbox, 1)
        self.setLayout(layout)

    def on_draw(self):
        key = str(self.slider_label.text())
        if self.dpi == 1:
            value = self.slider.value()
        else:
            value = (float)(self.slider.value()) / self.dpi
        self.textbox.setText(str(value))
        self.worker.set_value(key, value)


class SettingUi(QMainWindow):

    def __init__(self, worker):
        self.items = worker.get_settings()
        self.worker = worker

    def main(self):
        self.w = QWidget()
        self.w.resize(700, 150)
        self.w.setWindowTitle('Parameter')

        w_layout = QVBoxLayout()

        for key in self.items.keys():
            slider = SliderSetting(self.worker, key, self.items[key][0], self.items[key][1], self.items[key][2])
            w_layout.addWidget(slider)

        self.w.setLayout(w_layout)
        self.w.show()


class Worker():

    def __init__(self):
        self.settings = OrderedDict()
        self.settings['canny.th_low'] = [50, 0, 200]
        self.settings['canny.th_high'] = [150, 0, 600]
        self.settings['gaublue.filter_size'] = [5, 0, 20]
        self.settings['houghline.threshold'] = [100, 0, 200]
        self.settings['houghline.min_line_length'] = [100, 0, 600]
        self.settings['houghline.max_line_gap'] = [10, 0, 200]
        self.settings['extrapolation_lines.right_m_min'] = [-0.8, -1.0, 1.0]
        self.settings['extrapolation_lines.right_m_max'] = [-0.4, -1.0, 1.0]
        self.settings['extrapolation_lines.left_m_min'] = [0.4, -1.0, 1.0]
        self.settings['extrapolation_lines.left_m_max'] = [0.8, -1.0, 1.0]

    def get_settings(self):
        return self.settings

    def set_value(self, key, value):
        self.settings[key][0] = value

if __name__ == '__main__':
    worker = Worker()

    app = QApplication(sys.argv)
    mainApp = SettingUi(worker)
    mainApp.main()
    app.exec_()
