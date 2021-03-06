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
                 'xd', 'yd',
                 'vxd', 'vyd',
                 'x', 'y', 'z',
                 'vx', 'vy', 'vz',
                 'xz', 'yz', 'zz']
    data = Data(filename, labellist)
    return data


def plot_x(data):
    plt.figure()
    plt.plot(data.t, data.x, label='x')
    plt.plot(data.t, data.xz, label='xz')
    plt.plot(data.t, data.xd, label='xd')
    plt.legend()
    plt.title('COM / ZMP positions along x-axis')
    plt.xlabel('time [s]')
    plt.ylabel('x [m]')
    plt.grid(True)


def plot_vx(data):
    plt.figure()
    plt.plot(data.t, data.vxd, label='vxd', c='k', ls='--')
    plt.plot(data.t, data.vx, label='vx')
    plt.legend()
    plt.title('COM velocity along x-axis')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    plt.grid(True)


def plot_y(data):
    plt.figure()
    plt.plot(data.t, data.y, label='y')
    plt.plot(data.t, data.yz, label='yz')
    plt.plot(data.t, data.yd, label='yd')
    plt.legend()
    plt.title('COM / ZMP positions along y-axis')
    plt.xlabel('time [s]')
    plt.ylabel('y [m]')
    plt.grid(True)


def plot_vy(data):
    plt.figure()
    plt.plot(data.t, data.vyd, label='vyd', c='k', ls='--')
    plt.plot(data.t, data.vy, label='vy')
    plt.legend()
    plt.title('COM velocity along y-axis')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
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
    # plot_x(data)
    # plot_vx(data)
    plot_y(data)
    plot_vy(data)
    # plot_xy(data)
    plt.show()


if __name__ == '__main__':
    main()
