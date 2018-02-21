#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from subprocess import Popen
import os
import unittest


SAMPLE_DATA_FILENAME = "data/com_regulation_example.dat"
TESTING_DATA_FILENAME = "result.dat"
TEST_EXECUTABLE = "../com_regulation_example"


def load_sample_data():
    data = np.loadtxt(SAMPLE_DATA_FILENAME)
    return data


def generate_testing_data():
    cmd = [TEST_EXECUTABLE]
    with open(TESTING_DATA_FILENAME, 'w') as fobj:
        proc = Popen(cmd, stdout=fobj)
        proc.wait(timeout=1)
    data = np.loadtxt(TESTING_DATA_FILENAME)
    return data


class TestComRegulation(unittest.TestCase):

    def test_result(self):
        # check if sample data exists
        self.assertTrue(os.path.isfile(SAMPLE_DATA_FILENAME),
                        msg="sample data does not exist")

        # check if excutable exists
        self.assertTrue(os.path.isfile(TEST_EXECUTABLE),
                        msg="test executable does not exist")
        self.assertTrue(os.access(TEST_EXECUTABLE, os.X_OK),
                        msg="test executable is not executable")

        # check if generated data matches sample data
        sample = load_sample_data()
        testing = generate_testing_data()
        self.assertTrue(np.allclose(sample, testing),
                        msg="sample and testing data do not match")


if __name__ == '__main__':
    unittest.main()
