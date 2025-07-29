#!/usr/bin/env python

# import argparse
import numpy as np
import matplotlib.pyplot as plt

# args = argparse.ArgumentParser('')

# args.add_argument('files', nargs='+', help='List of files or file paths to process')

time, \
ok, \
sampling_state, \
temp_control_state, \
target_temp, \
current_temp, \
current_pressure, \
current_humidity, \
battery_voltage = np.genfromtxt(
    './data/cooler_structure_telemetry.csv',
    delimiter=',',
    unpack=True,
    names=True
)

time = time / 1000.0 # time is recorded in milliseconds

plt.plot(time, current_temp, label='Temperature')

plt.xlabel('Time (s)')
plt.ylabel('Temperature ($\\degree C$)')
plt.legend()

plt.show()
