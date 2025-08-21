import argparse
import numpy as np
import matplotlib.pyplot as plt
from accelerometer_arguments import accelerometer_arguments
from plot_data import plot_data

if __name__ == "__main__":
    parser = argparse.ArgumentParser("AccelerometerPlotter")
    parser.add_argument("files", nargs="+", help="List of files to process")
    args = accelerometer_arguments(parser)

    for filename in args.files:
        plt.figure()
        ax = plt.subplot()
        data = np.genfromtxt(filename, delimiter=",", names=True)
        plot_data(ax, data, **args.__dict__)
        ax.set_xlabel("Time (s)")
        ax.legend()
        plt.show()
