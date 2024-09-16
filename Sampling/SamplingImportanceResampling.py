import numpy as np
import matplotlib.pyplot as plt

k = [20, 100, 1000]
interval = [0, 15]

def normal_distribution(x, mu, sigma):
    return 1 / (np.sqrt(2*np.pi)*sigma) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)

def robot_postion(x):
    return 0.3 * normal_distribution(x, 2, 1) +  0.4 * normal_distribution(x, 5, 2) +  0.3 * normal_distribution(x, 9, 1)

def uniform_proposal(start, end, k):
    return np.random.uniform(start, end, k)

def importance_weight(samples):
    weights = np.array([])
    for i in samples:
        weights = np.append(weights, robot_postion(i))
    return weights / np.sum(weights)

def importance_resampling(k, interval):
    samples = uniform_proposal(interval[0], interval[1], k)
    weights = importance_weight(samples)
    return samples[np.random.choice(np.arange(k), k, p=weights)]

for j in range(3):
    for i in k:
        plt.hist(importance_resampling(i, interval), density=True, label=f'k={i}')
        x = np.array(np.arange(interval[0], interval[1], 0.01))
        y = robot_postion(x)
        plt.plot(x, y)
    plt.legend()
    plt.show()