# -*- coding: utf-8 -*-

from collections import OrderedDict


class ParamServer():
    # 'func_name.param_name'をkeyに持つdictionary
    params = OrderedDict()
    params['color.low_b'] = [87, 0, 255]
    params['color.low_g'] = [15, 0, 255]
    params['color.low_r'] = [76, 0, 255]
    params['color.high_b'] = [123, 0, 255]
    params['color.high_g'] = [255, 0, 255]
    params['color.high_r'] = [255, 0, 255]
    params['canny.th_low'] = [16, 1, 200]
    params['canny.th_high'] = [62, 1, 600]
    params['gaublue.filter_size'] = [5, 1, 20]
    params['houghline.threshold'] = [100, 1, 200]
    params['houghline.min_line_length'] = [100, 1, 600]
    params['houghline.max_line_gap'] = [5, 1, 200]
    params['extrapolation_lines.right_m_min'] = [-0.8, -1.0, 1.0]
    params['extrapolation_lines.right_m_max'] = [-0.2, -1.0, 1.0]
    params['extrapolation_lines.left_m_min'] = [0.2, -1.0, 1.0]
    params['extrapolation_lines.left_m_max'] = [0.8, -1.0, 1.0]

    @classmethod
    def get_all_params(cls):
        return cls.params

    @classmethod
    def get_param(cls, key):
        return cls.params[key][0]

    @classmethod
    def set_param(cls, key, value):
        cls.params[key][0] = value

    @classmethod
    def get_func_name(self, str):
        return str[:str.index('.')]

    @classmethod
    def get_param_name(self, str):
        return str[str.index('.') + 1:]
