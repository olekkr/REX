import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from localplanning_rrt import grid_occ, robot_models
from localplanning_rrt.rrt import RRT

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter


def main():

    path_res = 0.05
    map = grid_occ.GridOccupancyMap(low=(-1, 0), high=(1, 2), res=path_res)
    map.populate()

    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])   #

    rrt = RRT(
        start=[0, 0],
        goal=[0, 1.9],
        robot_model=robot,
        map=map,
        expand_dis=0.2,
        path_resolution=path_res,
        )
    
    show_animation = True
    metadata = dict(title="RRT Test")
    writer = FFMpegWriter(fps=15, metadata=metadata)
    fig = plt.figure()
    
    with writer.saving(fig, "rrt_test.mp4", 100):
        path = rrt.planning(animation=show_animation, writer=writer)

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")

            # Draw final path
            if show_animation:
                rrt.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.01)  # Need for Mac
                plt.show()
                writer.grab_frame()

if __name__ == "__main__":
    main()
