#!/usr/bin/env python
# -*- coding: utf-8 -*-


import matplotlib.pyplot as plt
import numpy as np


class Data(object):
    def __init__(self, filename, labellist):
        self.rawdata = np.loadtxt(filename)
        self.labellist = labellist
        self.make_data()

    def make_data(self):
        for i, label in enumerate(self.labellist):
            setattr(self, label, self.rawdata[:, i])


def load(filename):
    labellist = ['t', 'xd', 'vd', 'x', 'v']
    data = Data(filename, labellist)
    return data


def plot_x(data):
    plt.figure()
    plt.plot(data.t, data.xd, label='xd')
    plt.plot(data.t, data.x, label='x')
    plt.xlabel('time [s]')
    plt.ylabel('position [m]')
    plt.legend()
    plt.grid(True)


def main():
    import sys
    data = load(sys.argv[1])
    plot_x(data)
    plt.show()


if __name__ == '__main__':
    main()
