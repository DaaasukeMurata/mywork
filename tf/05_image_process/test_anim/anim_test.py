# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class AnimPlot():
    def __init__(self):
        self.fig = plt.figure()

    def plot(self, data):
        plt.cla()
        rand = np.random.randn(100)
        im = plt.plot(rand)

    def main(self):
        ani = animation.FuncAnimation(self.fig, self.plot, interval=100)
        plt.show()


if __name__ == '__main__':
    animplot = AnimPlot()
    animplot.main()
