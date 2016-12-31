# -*- coding: utf-8 -*-

from collections import OrderedDict


class Param(object):

    def __init__(self, value):
        self.value = value


class BoolParam(Param):

    def __init__(self, value):
        super(BoolParam, self).__init__(value)


class LinearParam(Param):

    def __init__(self, value, lower, upper):
        super(LinearParam, self).__init__(value)
        self.lower = lower
        self.upper = upper


class ParamServer():
    # 'func_name.param_name'をkeyに持つdictionary
    # [value, 下限, 上限]
    __params = OrderedDict()
    __params['system.color_filter'] = BoolParam(value=1)
    __params['system.to_gray'] = BoolParam(value=1)
    __params['system.blur'] = BoolParam(value=0)
    __params['system.detect_edge'] = BoolParam(value=1)
    __params['color.low_b'] = LinearParam(value=0, lower=0, upper=255)
    __params['color.low_g'] = LinearParam(value=0, lower=0, upper=255)
    __params['color.low_r'] = LinearParam(value=34, lower=0, upper=255)
    __params['color.high_b'] = LinearParam(value=255, lower=0, upper=255)
    __params['color.high_g'] = LinearParam(value=255, lower=0, upper=255)
    __params['color.high_r'] = LinearParam(value=255, lower=0, upper=255)
    __params['gaublur.filter_size'] = LinearParam(value=5, lower=1, upper=11)
    __params['canny.th_low'] = LinearParam(value=39, lower=1, upper=255)
    __params['canny.th_high'] = LinearParam(value=150, lower=1, upper=255)
    __params['houghline.threshold'] = LinearParam(value=50, lower=1, upper=200)
    __params['houghline.min_line_length'] = LinearParam(value=100, lower=1, upper=100)
    __params['houghline.max_line_gap'] = LinearParam(value=5, lower=1, upper=100)
    __params['extrapolation_lines.right_m_min'] = LinearParam(value=-0.8, lower=-1.0, upper=1.0)
    __params['extrapolation_lines.right_m_max'] = LinearParam(value=-0.2, lower=-1.0, upper=1.0)
    __params['extrapolation_lines.left_m_min'] = LinearParam(value=0.2, lower=-1.0, upper=1.0)
    __params['extrapolation_lines.left_m_max'] = LinearParam(value=0.8, lower=-1.0, upper=1.0)

    value_changed_func = None

    # define
    TYPE_NONE = 0
    TYPE_BOOL = 1
    TYPE_LINEAR = 2

    @classmethod
    def get_all_params(cls):
        return cls.__params

    @classmethod
    def get_value(cls, key):
        return cls.__params[key].value

    @classmethod
    def set_value(cls, key, value):
        cls.__params[key].value = value
        cls.__do_cb_value_changed()

    @classmethod
    def get_type(cls, key):
        if isinstance(cls.__params[key], BoolParam):
            return cls.TYPE_BOOL
        elif isinstance(cls.__params[key], LinearParam):
            return cls.TYPE_LINEAR
        else:
            return cls.TYPE_NONE

    @classmethod
    def get_lower(cls, key):
        return cls.__params[key].lower

    @classmethod
    def get_upper(cls, key):
        return cls.__params[key].upper

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
