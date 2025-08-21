import argparse
from telemetry_arguments import telemetry_arguments
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
from plot_data import plot_data
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser("TelemetryAllPlotter")
    parser.add_argument("dir", help="Directory containing telemetry data files.")
    parser.add_argument('--limits', '-l', nargs=2, type=int, required=False)
    args = telemetry_arguments(parser)

    n = 0
    data_list = []
    while os.path.isfile(f"{args.dir}/DATA{n}_telemetry.csv"):
        data_list.append(np.genfromtxt(f"{args.dir}/DATA{n}_telemetry.csv", delimiter=",", names=True))
        n += 1

    runs = [data for data in data_list if len(data['time']) != 0]

    fig = plt.figure()
    axs = gs.GridSpec(1, len(runs), wspace=0, width_ratios=[data['time'][-1] for data in runs])

    for i, data in enumerate(runs):
        ax = plt.subplot(axs[0, i])
        plot_data(ax, data, **args.__dict__)
        if i != 0:
            ax.yaxis.set_visible(False)
        if args.limits != None:
            ax.set_ylim(args.limits[0], args.limits[1])

    fig.supxlabel('Time (s)')
    plt.show()
