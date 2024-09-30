import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes

from constants import Constants

# AN occupancy grid implementation

GRID_FIDELITY = 1000  # size of each grid square in mm
LANDMARK_DIAMETER = 450


def empty_grid():
    return np.zeros((5, 5)).astype(int)


# from camera position to robo position
def from_cam_pos(x):
    return x[0], x[2] - Constants.Robot.DIAMETER / 2


def from_grid_pos(x):
    return x * GRID_FIDELITY


def draw_radius(px, py, r, grid):
    def pred(i, j):
        p = np.sqrt((from_grid_pos(i) - px) ** 2 + (from_grid_pos(j) - py) ** 2) < r
        return p

    p = np.fromfunction(pred, grid.shape)

    grid = p | grid
    return grid


def draw_landmarks(landmarks, grid):
    for xyz in landmarks:
        x, y = from_cam_pos(xyz)
        grid = draw_radius(x, y, Constants.Robot.DIAMETER + LANDMARK_DIAMETER, grid)
    return grid


def robo_pos_avail(pos, grid):
    pass


def show_grid(grid, robo_pos, axes: Axes):
    axes.imshow(grid.astype(int))
    axes.scatter([robo_pos[0]], [robo_pos[1]])


landmarks = None

grid = np.zeros((5, 5)).astype(int)


# test case:
