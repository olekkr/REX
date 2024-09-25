import math
from random import choice

import matplotlib.pyplot as plt

SAMPLE_RANGE = range(0, 15 + 1)


def normal_dist(x: float, mean: float, deviation: float):
    return (1 / (math.sqrt(2 * math.pi) * deviation)) * math.exp(
        -0.5 * (((x - mean) ** 2) / (deviation**2))
    )


def q(x: int):
    return (
        0.3 * normal_dist(x, 2, 1)
        + 0.4 * normal_dist(x, 5, 2)
        + 0.3 * normal_dist(x, 9, 1)
    )


def gen_samples(k: int, sample_range=SAMPLE_RANGE):
    return [choice(sample_range) for _ in range(k)]
