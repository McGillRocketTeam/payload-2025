import argparse
from telemetry_arguments import telemetry_arguments
import numpy as np
import matplotlib.pyplot as plt
from plot_data import plot_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser("TelemetryPlotter")
    parser.add_argument("files", nargs="+", help="List of files to process")
    args = telemetry_arguments(parser)

    for filename in args.files:
        plt.figure()
        ax = plt.subplot()
        data = np.genfromtxt(filename, delimiter=",", names=True)
        plot_data(ax, data, **args.__dict__)
        ax.set_xlabel("Time (s)")
        ax.legend()
        plt.show()
