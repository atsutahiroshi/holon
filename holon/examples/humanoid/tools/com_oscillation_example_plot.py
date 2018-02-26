#!/usr/bin/env python3
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
    labellist = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'xz', 'yz', 'zz']
    data = Data(filename, labellist)
    return data


def plot_y(data):
    plt.figure()
    plt.plot(data.t, data.y, label='y')
    plt.plot(data.t, data.yz, label='yz')
    plt.legend()
    plt.title('Positions of COM and ZMP')
    plt.xlabel('time [s]')
    plt.ylabel('y [m]')
    plt.grid(True)


def plot_yvy(data):
    plt.figure()
    plt.plot(data.y, data.vy, label='y-vy')
    plt.plot(data.yz, data.vy, label='yz-vy')
    plt.legend()
    plt.title('Phase portrait of COM along y-axis')
    plt.xlabel('position [m]')
    plt.ylabel('velocity [m/s]')
    plt.grid(True)


def main():
    import sys
    data = load(sys.argv[1])
    plot_y(data)
    plot_yvy(data)
    plt.show()


if __name__ == '__main__':
    main()
