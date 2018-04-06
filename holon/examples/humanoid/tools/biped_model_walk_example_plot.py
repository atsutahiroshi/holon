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
    labellist = ['t',
                 # 'vxd', 'vyd',
                 'x', 'y', 'z',
                 'vx', 'vy', 'vz',
                 'xz', 'yz', 'zz']
    data = Data(filename, labellist)
    return data


def plot_x(data):
    plt.figure()
    plt.plot(data.t, data.x, label='x')
    plt.plot(data.t, data.xz, label='xz')
    plt.legend()
    plt.title('COM / ZMP positions along x-axis')
    plt.xlabel('time [s]')
    plt.ylabel('x [m]')
    plt.grid(True)


def plot_y(data):
    plt.figure()
    plt.plot(data.t, data.y, label='y')
    plt.plot(data.t, data.yz, label='yz')
    plt.legend()
    plt.title('COM / ZMP positions along y-axis')
    plt.xlabel('time [s]')
    plt.ylabel('y [m]')
    plt.grid(True)


def plot_xy(data):
    plt.figure()
    plt.plot(data.x, data.y, label='trajectory of COM')
    plt.plot(data.xz, data.yz, label='trajectory of ZMP')
    plt.legend()
    plt.title('Trajectories of COM / ZMP')
    plt.xlabel('x position [m]')
    plt.ylabel('y position [m]')
    plt.grid(True)


def main():
    import sys
    data = load(sys.argv[1])
    plot_x(data)
    plot_y(data)
    plot_xy(data)
    plt.show()


if __name__ == '__main__':
    main()
