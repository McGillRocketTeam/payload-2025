import numpy as np
import matplotlib.pyplot as plt

data = 3.3 * (np.loadtxt('data/adc.csv', delimiter=',').transpose() - 2048) / 4096 / 2
time = np.array(range(1024)) * (1 / 10e3)

plt.plot(time, data, linewidth=1)

plt.savefig(f'graphs/adc.png')
plt.show()
