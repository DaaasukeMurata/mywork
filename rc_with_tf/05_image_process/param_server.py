# -*- coding: utf-8 -*-

from collections import OrderedDict


class ParamServer():
    # 'func_name.param_name'をkeyに持つdictionary
    # [value, 下限, 上限]
    params = OrderedDict()
    params['system.color_filter'] = [0, 0, 1]
    params['system.to_gray'] = [1, 0, 1]
    params['system.blur'] = [0, 0, 1]
    params['system.detect_edge'] = [1, 0, 1]
    params['color.low_b'] = [87, 0, 255]
    params['color.low_g'] = [15, 0, 255]
    params['color.low_r'] = [76, 0, 255]
    params['color.high_b'] = [123, 0, 255]
    params['color.high_g'] = [255, 0, 255]
    params['color.high_r'] = [255, 0, 255]
    params['gaublur.filter_size'] = [5, 1, 20]
    params['canny.th_low'] = [39, 1, 255]
    params['canny.th_high'] = [150, 1, 255]
    params['houghline.threshold'] = [50, 1, 200]
    params['houghline.min_line_length'] = [100, 1, 100]
    params['houghline.max_line_gap'] = [5, 1, 100]
    params['extrapolation_lines.right_m_min'] = [-0.8, -1.0, 1.0]
    params['extrapolation_lines.right_m_max'] = [-0.2, -1.0, 1.0]
    params['extrapolation_lines.left_m_min'] = [0.2, -1.0, 1.0]
    params['extrapolation_lines.left_m_max'] = [0.8, -1.0, 1.0]

    value_changed_func = None

    @classmethod
    def get_all_params(cls):
        return cls.params

    @classmethod
    def get_param(cls, key):
        return cls.params[key][0]

    @classmethod
    def get_lower(cls, key):
        return cls.params[key][1]

    @classmethod
    def get_upper(cls, key):
        return cls.params[key][2]

    @classmethod
    def set_param(cls, key, value):
        cls.params[key][0] = value
        cls.__do_cb_value_changed()

    @classmethod
    def get_func_name(self, str):
        return str[:str.index('.')]

    @classmethod
    def get_param_name(self, str):
        return str[str.index('.') + 1:]

    @classmethod
    def add_cb_value_changed(cls, func):
        cls.value_changed_func = func

    @classmethod
    def __do_cb_value_changed(cls, *args):
        if cls.value_changed_func is not None:
            cls.value_changed_func(*args)
