import numpy as np
import matplotlib.pyplot as plt
import random

k = [20, 100, 1000]
interval = [0, 15]

#e = 2.71828

def N(x, mu, sigma):
    return (1 / np.sqrt(2) * sigma) * np.e ** (-0.5 * (x - mu) ** 2 / sigma ** 2)

def P(x):
    return (0.3 * N(x, 2, 1) + 
            0.4 * N(x, 5, 2) + 
            0.3 * N(x, 9, 1))

def Q(x): 
    rng = np.random.default_rng()
    
    
print(Q(interval))

for i in range(1):
    x = np.arange(interval[0], interval[1], 0.01)
    y = P(x)
    #plt.hist(sample(i, interval))
    plt.plot(x, y)
    plt.show()


# x = np.arange(interval[0], interval[1])
# y = P(x)
# plt.plot(x, y)
# plt.show()