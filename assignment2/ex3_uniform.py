# %%
import math
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# %%
SAMPLE_MIN = 0
SAMPLE_MAX = 15
SAMPLE_RANGE = (SAMPLE_MIN, SAMPLE_MAX)
num_samples = 30


# %%
def normal_dist(x: float, mean: float, deviation: float):
    return (1 / (math.sqrt(2 * math.pi) * deviation)) * math.exp(
        -0.5 * (((x - mean) ** 2) / (deviation**2))
    )

def p(x: int):
    return (
        0.3 * normal_dist(x, 2, 1)
        + 0.4 * normal_dist(x, 5, 2)
        + 0.3 * normal_dist(x, 9, 1)
    )


# %%
def gen_uniform_samples(sample_max: int=SAMPLE_MAX, k=num_samples, seed=0):
    rng = np.random.default_rng(seed)

    return [rng.random()*sample_max for _ in range(k)]


# %%
def resample(samples, k=num_samples):
    target_density = np.array([p(i) for i in samples])  # np arrays allow division
    weights = target_density / samples
    normalized_weights = weights / np.sum(weights)

    return [samples[resampled_idx] for resampled_idx in np.random.choice(np.arange(k), k, p=normalized_weights)]


# %%

def print_distr(samples, normalize: bool = False):
    int_samples = [round(v) for v in samples]

    distr_dict = {v:0. for v in range(min(int_samples),max(int_samples)+1)}
    for sample in int_samples:
        distr_dict.setdefault(sample, 0)
        distr_dict[sample] += 1

    print(distr_dict)
    if normalize:
        total = sum(distr_dict.values())
        for key in distr_dict:
            distr_dict[key] /= total
        print(distr_dict)

    return distr_dict

# %%
x = np.arange(0,15,0.01)
y = [p(idx) for idx in x]
plt.plot(x, y)

for alpha, histtype, k in zip((1,0.7,1, 1), ("bar","bar","step", "step"),(20,100,1000, 10000)):
    samples = gen_uniform_samples(k=k, seed=int(datetime.now().timestamp()))
    plt.hist(samples, density=True, label=f"samples, k={k}",alpha=0.9)
    resampled = resample(samples, k=k)
    print_distr(samples, normalize=True)
    rd = print_distr(resampled, normalize=True)
    plt.hist(resampled, density=True, label=f"resampled, k={k}", histtype=histtype, alpha=alpha)

plt.legend()
plt.show()


