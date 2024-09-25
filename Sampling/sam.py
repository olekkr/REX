import numpy as np
import matplotlib.pyplot as plt
import random
import math

k = [20, 100, 1000]
interval = [0, 15]

def N(x, mu, sigma):
    return (1 / (math.sqrt(2 * np.pi) * sigma)) * np.exp((-0.5 * ((x - mu) ** 2) / sigma ** 2))

def P(x):
    return 0.3 * N(x, 2, 1) + 0.4 * N(x, 5, 2) + 0.3 * N(x, 9, 1)

def Q(k):
    return [random.uniform(interval[0], interval[1]) for _ in range(k)]

def sample(sample):
    samples =  [P(x) for x in sample]
    weight = np.array(samples) / np.array(sample)
    norm_weight = weight / np.sum(weight)
    return np.array([sample[v] for v in np.random.choice(np.arange(len(sample)), len(sample), replace=True, p=norm_weight)])

for i in k:
    x = np.arange(interval[0], interval[1], 0.01)
    y = P(x)
    plt.hist(sample(Q(i)),density=True)
plt.plot(x, y)
plt.show()

# def ex2_sample(sample):
#     samples = [N(x, 5, 4) for x in sample]
#     weight = np.array(samples) / np.array(sample)
#     norm_weight = weight / np.sum(weight)
#     return np.array([sample[v] for v in np.random.choice(np.arange(len(sample)), len(sample), replace=True, p=norm_weight)])
    
# for i in k:
#     x = np.arange(interval[0], interval[1], 0.01)
#     y = N(x, 5, 4)
#     plt.hist(ex2_sample(Q(i)), density=True)
#     plt.plot(x, y)
#     plt.show()