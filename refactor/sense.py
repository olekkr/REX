import math
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from camera.cam import Camera
from constants import Constants
if Constants.Robot.INCLUDE is False:
    straight_move = lambda dist: None
    rotate_move = lambda frac: None
else:
    from calibrate import rotate_move, straight_move
from localplanning_rrt import grid_occ, robot_models
from localplanning_rrt.rrt import RRT
from refactor import local_planning

dimensions = Constants.PID.SCREEN_RESOLUTION

FocalLength = Constants.PID.FOCALLENGTH

markerHeight = Constants.PID.MARKER_HEIGHT  # m
FPS = 5

picam2 = Camera()

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()


CameraMatrix = Constants.PID.CAMERA_MATRIX
DistortionCoefficient = Constants.PID.DISTORTION_COEFFICIENT

print("using dimention:", dimensions)
print("using FocalLength:", FocalLength)


def init():
    # grid init
    return local_planning.empty_grid()


def sense(grid):  # map,
    return sense_camera(grid)


def preview_marker(corners, im):
    if Constants.PID.ENABLE_PREVIEW:
        if corners:
            try:
                for corner in corners:
                    picam2.preview(cv2.aruco.drawDetectedCornersCharuco(im, corner))
            except:
                pass
        else:
            picam2.preview(im)


def sense_camera(grid):
    # capture RGB:
    im = picam2.take_image(enable_preview=False)

    # capture AruCo Corners
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=im, dictionary=arucoDict)

    if Constants.PID.ENABLE_PREVIEW:
        if corners:
            try:
                for corner in corners:
                    picam2.preview(cv2.aruco.drawDetectedCornersCharuco(im, corner))
            except:
                pass
        else:
            picam2.preview(im)

    # get Markers in camera coordinate system
    _rt, tv, _objs = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        markerHeight,
        CameraMatrix,
        DistortionCoefficient,
    )
    if tv is None:
        tv = []

    for t in tv:
        grid = local_planning.draw_landmarks(t, grid)

    return grid


class CustomGridOccupancyMap(grid_occ.GridOccupancyMap):
    def populate_real(self):
        """
        generate a grid map with some circle shaped obstacles
        """
        global start_time2
        # capture RGB:
        im = picam2.take_image(enable_preview=False)

        # capture AruCo Corners
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image=im, dictionary=arucoDict)

        preview_marker(corners, im)

        _rt, tv, _objs = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            markerHeight,
            CameraMatrix,
            DistortionCoefficient,
        )

        if tv is None:
            tv = []

        origins = np.array([(x / 1000, z / 1000) for t in tv for x, y, z in t])
        radius = (Constants.Obstacle.SHAPE_RADIUS + Constants.Robot.RADIUS * 1.2) / 1000

        # fill the grids by checking if the grid centroid is in any of the circle
        for i in range(self.n_grids[0]):
            for j in range(self.n_grids[1]):
                centroid = np.array(
                    [
                        self.map_area[0][0] + self.resolution * (i + 0.5),
                        self.map_area[0][1] + self.resolution * (j + 0.5),
                    ]
                )
                for o in origins:
                    if time.time() - start_time2 > 0.1:
                        # print(
                        #     centroid,
                        #     np.linalg.norm(centroid - o),
                        #     radius,
                        #     np.linalg.norm(centroid - o) <= radius,
                        # )
                        start_time2 = time.time()
                    if np.linalg.norm(centroid - o) <= radius:
                        self.grid[i, j] = 1
                        break


def angle_between(current_pos, goal_pos, current_ang=90.0):
    ang1 = np.arctan2(*current_pos[::-1])
    ang2 = np.arctan2(*goal_pos[::-1])
    return_ang = np.rad2deg((ang2 - ang1) % (2 * np.pi))
    return (return_ang - current_ang) % 360

def center_points_to_robot(point, current_angle):
    robot_orientation_rad = np.deg2rad(current_angle)

    shift_x = Constants.Robot.RADIUS * np.cos(robot_orientation_rad)
    shift_y = Constants.Robot.RADIUS * np.sin(robot_orientation_rad)

    return point[0] - shift_x, point[1] - shift_y
    

def gen_instructions_from_points(points):

    path_w_angles = [(0.0, 0.0, 90.0)]
    angles_and_dist = []
    for x, y in path_from_start[1:]:
        current_x, current_y, current_angle = path_w_angles[-1]
        centered_x, centered_y = center_points_to_robot((x,y), current_angle)
        vec = np.array([centered_x, centered_y]) - np.array([current_x, current_y])
        target_angle = np.rad2deg(np.arctan2(vec[1], vec[0])) % 360
        turn_angle = current_angle - target_angle
        path_w_angles.append((centered_x, centered_y, target_angle))
        angles_and_dist.append(
            (
                turn_angle,
                np.linalg.norm(
                    np.array([current_x, current_y]) - np.array([centered_x, centered_y])
                ),
            )
        )
    return angles_and_dist

if __name__ == "__main__":
    grid = init()
    path_res = 0.05

    map = CustomGridOccupancyMap(low=(-1, 0), high=(1, 2), res=path_res)

    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])  #
    start_time = time.time()
    start_time2 = time.time()
    metadata = dict(title="RRT Test")

    writer = FFMpegWriter(fps=1000, metadata=metadata)
    fig = plt.figure()
    show_animation = False
    rrt = RRT(
        start=np.array([0, 0]),
        goal=[0, 1.9],
        robot_model=robot,
        map=map,
        expand_dis=0.2,
        path_resolution=path_res,
    )
    while True:
        map.populate_real()

        # fig = plt.figure()
        if time.time() - start_time < 1:
            continue

        with writer.saving(fig, "rrt_test.mp4", 100):
            path = rrt.planning(animation=show_animation, writer=writer)

            if path is None:
                print("Cannot find path")
            else:
                print("found path!!")

                # Draw final path
                if show_animation:
                    rrt.draw_graph()
                    plt.plot([x for (x, y) in path], [y for (x, y) in path], "-r")
                    plt.grid(True)
                    plt.pause(0.01)  # Need for Mac
                    plt.show()
                    writer.grab_frame()
                else:
                    path_from_start = path[::-1]
                    angles_and_dist = gen_instructions_from_points(path_from_start)
                    rrt.draw_graph()
                    plt.plot([x for (x, y) in path], [y for (x, y) in path], "-r")
                    plt.grid(True)
                    plt.pause(0.01)  # Need for Mac
                    writer.grab_frame()
                    plt.show(block=False)


                    for angle, dist in angles_and_dist:
                        # input(f"enter to rotate {angle}")
                        rotate_move(frac=angle / 90)
                        # input(f"enter to move {dist}")
                        straight_move(dist)
                        time.sleep(1)

                    exit()
