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
                 'xd', 'yd', 'zd',
                 'x', 'y', 'z',
                 'vx', 'vy', 'vz',
                 'xz', 'yz', 'zz']
    data = Data(filename, labellist)
    return data


def plot_z(data):
    plt.figure()
    plt.plot(data.t, data.zd, label='zd', c='k', ls='--')
    plt.plot(data.t, data.z, label='z')
    plt.legend()
    plt.title('COM regulation along vertical direction')
    plt.xlabel('time [s]')
    plt.ylabel('z [m]')
    plt.grid(True)


def main():
    import sys
    data = load(sys.argv[1])
    plot_z(data)
    plt.show()


if __name__ == '__main__':
    main()
