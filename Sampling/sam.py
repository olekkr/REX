import numpy as np
import matplotlib.pyplot as plt
import random

k = [20, 100, 1000]
interval = [0, 15]

#e = 2.71828

def N(x, p, q):
    return (1 / np.sqrt(2 * np.pi) * q) *  np.e ** ((-0.5 * (x - p)**2) / (q ** 2))

def P(x):
    return 0.3 * N(x, 2.0, 1.0) + 0.4 * N(x, 5.0, 2.0) + 0.3 * N(x, 9.0, 1.0)

def Q(x):
    return random.randint(interval[0], interval[1])




x = np.arange(interval[0], interval[1])
y = P(x)
plt.plot(x, y)
plt.show()