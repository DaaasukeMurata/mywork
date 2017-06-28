#!/usr/bin/env python
# coding: utf-8

import sys
import time
import matplotlib.pyplot as plt


class PIDControl():

    def __init__(self, delta):
        self.before_val = 0
        self.now_val = val = 0
        self.integral = 0
        self.integral_time = 0.01
        self.DELTA_T = delta

    def control(self, val, target):
        KP = 6.667
        KI = KP / self.integral_time
        KD = KP * self.DELTA_T

        self.before_val = self.now_val
        self.now_val = target - val

        self.integral += (self.before_val + self.now_val) / 2.0 * self.DELTA_T
        self.integral_time += self.DELTA_T

        p = KP * self.now_val
        i = KI * self.integral
        d = KD * (self.now_val - self.before_val) / self.DELTA_T

        # return p + i + d
        return max(min(100, p + i + d), 0)


# 0～100の位置で移動
class CarStub():

    def __init__(self, default_pos):
        self.pos = default_pos

    def moov(self, distance):
        wk_pos = self.pos + (distance / 10.0)
        # self.pos = max(min(100, wk_pos), 0)
        self.pos = wk_pos


def main():
    argvs = sys.argv
    if len(argvs) < 1:
        print('[usage]python pid_test.py [default pos]')
        sys.exit(0)

    Car = CarStub(float(sys.argv[1]))
    PIDC = PIDControl(0.1)

    pos_list = [Car.pos]
    val_list = [50]

    for i in range(200):
        val = PIDC.control(Car.pos, 50)
        Car.moov(val - 50)
        val_list.append(val)
        pos_list.append(Car.pos)
        print("val:%6.2f" % val, "  Car.pos:%6.2f" % Car.pos)
        # time.sleep(0.1)

    plt.plot(val_list)
    plt.plot(pos_list)
    plt.show()


if __name__ == '__main__':

    main()
