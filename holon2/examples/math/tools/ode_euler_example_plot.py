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
    labellist = ['x', 'y']
    data = Data(filename, labellist)
    return data


def plot_xy(data):
    plt.figure()
    plt.plot(data.x, data.y, label='solution')
    plt.legend()
    plt.grid(True)


def main():
    import sys
    data = load(sys.argv[1])
    plot_xy(data)
    plt.show()


if __name__ == '__main__':
    main()
