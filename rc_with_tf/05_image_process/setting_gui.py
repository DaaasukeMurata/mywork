# -*- coding: utf-8 -*-

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from collections import OrderedDict
import sys


class SliderSetting(QWidget):

    def __init__(self, worker, name, init_value, range_low, range_high, parent=None):
        super(SliderSetting, self).__init__()
        self.worker = worker
        self.name = name
        self.setup_ui(name, init_value, range_low, range_high)

    def setup_ui(self, name, init_value, range_low, range_high):
        # floatの場合、スライドの値1で、反映値0.1とする
        self.dpi = 1
        if isinstance(init_value, float):
            self.dpi = 10

        self.slider_label = QLabel(self.get_paramname(name))
        self.slider = QSlider(Qt.Horizontal)  # スライダの向き
        self.slider.setRange(range_low * self.dpi, range_high * self.dpi)  # スライダの範囲
        self.slider.setValue(init_value * self.dpi)
        self.slider.setTickPosition(QSlider.TicksAbove)  # メモリの位置
        self.connect(self.slider, SIGNAL('valueChanged(int)'), self.on_draw)
        self.textbox = QLineEdit()
        self.textbox.setText(str(init_value))

        layout = QHBoxLayout()
        layout.setMargin(0)

        layout.addWidget(self.slider_label, 3)
        layout.addWidget(self.slider, 6)
        layout.addWidget(self.textbox, 1)
        self.setLayout(layout)

    def on_draw(self):
        key = self.name
        if self.dpi == 1:
            value = self.slider.value()
        else:
            value = (float)(self.slider.value()) / self.dpi
        self.textbox.setText(str(value))
        self.worker.set_param(key, value)
        self.worker.redraw()

    def get_paramname(self, str):
        return str[str.index('.') + 1:]


class SettingWidget(QWidget):

    def __init__(self, func_name, worker, items, parent=None):
        super(SettingWidget, self).__init__()
        self.worker = worker
        self.items = items
        self.name = func_name
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        # layout.setMargin(0)
        layout.setAlignment(Qt.AlignTop)

        # func_name.から始まるitemの設定項目を作成
        for key in self.items.keys():
            key_func = self.get_funcname(key)
            if (key_func == self.name):
                slider = SliderSetting(self.worker, key, self.items[key][0], self.items[key][1], self.items[key][2])
                layout.addWidget(slider)

        self.setLayout(layout)

    def get_funcname(self, str):
        return str[:str.index('.')]


class SettingUi(QMainWindow):

    def __init__(self, worker):
        super(SettingUi, self).__init__()
        self.worker = worker
        self.items = self.worker.get_params()  # 'Func.Param'のkey名のOrderedDict
        self.init_ui()

    def init_ui(self):
        self.w = QWidget()
        self.w.resize(500, 150)
        self.w.setWindowTitle('Parameter')

        # tabにするwidget作成
        tabs_dict = OrderedDict()
        for key in self.items.keys():
            key_func = self.get_funcname(key)

            # 新しいfuncが見つかったとき、widget作成
            if key_func not in tabs_dict:
                setting_widget = SettingWidget(key_func, self.worker, self.items)
                tabs_dict[key_func] = setting_widget

        tab = QTabWidget()

        for key in tabs_dict.keys():
            tab.addTab(tabs_dict[key], key)

        w_layout = QHBoxLayout()
        w_layout.addWidget(tab)

        self.w.setLayout(w_layout)
        self.w.show()

    def get_funcname(self, str):
        return str[:str.index('.')]
