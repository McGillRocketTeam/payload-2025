import matplotlib.axes as axes
import numpy as np


def plot_data(ax: axes.Axes, data: np.ndarray, **kwargs):
    for key, value in kwargs.items():
        if key == "files" or key == "dir" or key == "limits":
            continue

        if value:
            ax.plot(data["time"] / 1000.0, data[key], label=key)
