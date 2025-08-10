import matplotlib.pyplot as plt
import numpy as np


def plot_data(data: np.ndarray, **kwargs):
    for key, value in kwargs.items():
        if key == "files":
            continue

        if value:
            plt.plot(data["time"] / 1000.0, data[key], label=key)

    plt.xlabel("Time (s)")
    plt.legend()
    plt.show()
