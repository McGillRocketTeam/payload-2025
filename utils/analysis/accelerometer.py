import argparse
import numpy as np
import matplotlib.pyplot as plt
from plot_data import plot_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser("TelemetryPlotter")
    parser.add_argument("files", nargs="+", help="List of files to process")
    parser.add_argument("-x", action="store_true")
    parser.add_argument("-y", action="store_true")
    parser.add_argument("-z", action="store_true")
    args = parser.parse_args()

    for filename in args.files:
        data = np.genfromtxt(filename, delimiter=",", names=True)
        plot_data(data, **args.__dict__)
