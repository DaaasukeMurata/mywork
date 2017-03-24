#!/usr/bin/env python
# coding: UTF-8

import random
import numpy as np
import matplotlib.pyplot as plt

# wk_list = np.empty((0, 2))
# for index in range(10):
#     y = random.randint(0, 100)
#     wk_list = np.append(wk_list, np.array([[index, y]]), axis=0)

# plt.plot(wk_list[0], wk_list[1], 'ro')
# plt.show()

x = np.arange(-3, 3, 0.1)
y = np.sin(x)
y2 = np.cos(x)

plt.plot(x, y)
plt.plot(x, y2)

plt.show()
