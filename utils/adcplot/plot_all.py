import numpy as np
import matplotlib.pyplot as plt

data = 3.3 * (np.loadtxt('data/adc.csv', delimiter=',').transpose() - 2048) / 4096 / 2
time = np.arange(2048) * (1 / 10e3) # Assuming a sampling rate of 10 kHz

for i, d in enumerate(data):
    plt.plot(time, d, linewidth=1)
    
    plt.savefig(f'graphs/{i}.png')
    plt.show()
