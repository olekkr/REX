import numpy as np 
import matplotlib.pyplot as plt
from math import sqrt
from constants import *

# AN occupancy grid implementation 

GRID_FIDELITY = 1000 # size of each grid square in mm 
LANDMARK_DIAMETER = 450


def empty_grid():
    return np.zeros((100,100)).astype(int)

# from camera position to robo position
def from_cam_pos(x): 
    return x[0], x[2] - ROBOT_DIAMETER/2

def from_grid_pos(x):
    return x * GRID_FIDELITY

def draw_radius(px, py, r, grid):
    def pred(i,j) : 
        p = np.sqrt((from_grid_pos(i)-px)**2 + (from_grid_pos(j)-py)**2) < r
        return p

    p = np.fromfunction(pred, grid.shape)
    
    grid = (p | grid)
    return grid

def draw_landmarks(landmarks, grid):
    for xyz in landmarks:
        x,y = from_cam_pos(xyz)
        draw_radius(x,y, ROBOT_DIAMETER+LANDMARK_DIAMETER, grid)

def robo_pos_avail(pos, grid):
    pass

def show_grid(grid, robo_pos):
    plt.imshow(grid.astype(int))
    plt.scatter([robo_pos[0]],[robo_pos[1]])
    plt.show()


landmarks = None

grid = np.zeros((100,100)).astype(int)


# test case: 

