import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('data/adc.csv', delimiter=',').transpose()
time = data[0]

for i, d in enumerate(data[1:]):
    plt.plot(time, d, linewidth=1)
    
    plt.savefig(f'graphs/{i}.png')
    plt.show()
