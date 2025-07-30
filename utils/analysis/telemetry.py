import argparse
import numpy as np
import matplotlib.pyplot as plt
from plot_data import plot_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser("TelemetryPlotter")
    parser.add_argument("files", nargs="+", help="List of files to process")
    parser.add_argument("-o", "--ok", action="store_true")
    parser.add_argument("-s", "--sampling-state", action="store_true")
    parser.add_argument("-c", "--temperature-control", action="store_true")
    parser.add_argument("-T", "--target-temp", action="store_true")
    parser.add_argument("-t", "--current-temp", action="store_true")
    parser.add_argument("-p", "--current-pressure", action="store_true")
    parser.add_argument("-H", "--current-humidity", action="store_true")
    parser.add_argument("-v", "--battery-voltage", action="store_true")
    args = parser.parse_args()

    for filename in args.files:
        data = np.genfromtxt(filename, delimiter=",", names=True)
        plot_data(data, **args.__dict__)
