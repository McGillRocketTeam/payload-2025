#!/usr/bin/env python

# import argparse
import numpy as np
import matplotlib.pyplot as plt

# args = argparse.ArgumentParser('')

# args.add_argument('files', nargs='+', help='List of files or file paths to process')

time, x, y, z = np.genfromtxt(
    './data/accelerometer_hardware_test/DATA21_accelerometer.csv',
    delimiter=',',
    max_rows=2048 * 1000,
    unpack=True,
    names=True
)

time = time / 1000.0 # time is recorded in milliseconds

plt.plot(time, x, label='x')
plt.plot(time, y, label='y')
plt.plot(time, z, label='z')

plt.xlabel('Time (s)')
plt.ylabel('Acceleration (g)')
plt.legend()

plt.show()
