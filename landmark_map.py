import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from camera.picam import Camera

import robot
from constants import Constants

CAMERA_MATRIX, DISTORTION_COEFFICIENT, MARKER_HEIGHT = Constants.PID.CAMERA_MATRIX, Constants.PID.DISTORTION_COEFFICIENT, Constants.PID.MARKER_HEIGHT

arlo = robot.Robot()

speed = 40
tspeed_slow = 32
tspeed = 32
aruco = False
image = "aruco.png"
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
preview_downscale = 2
imageSize = (1920 // 2**preview_downscale, 1080 // 2**preview_downscale)
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = 5
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds


picam2 = Camera()

arucoParams = cv2.aruco.DetectorParameters()


def detect(image_inp):
    # print(image_inp.shape)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=image_inp, dictionary=arucoDict)
    return corners, ids, rejected


def drive_straight(i: int = 1):
    print("Found target")
    arlo.go_diff(speed, speed, 1, 1)
    time.sleep(1)
    return i + 1


def turn_left(i):
    print("moving left")
    arlo.go_diff(tspeed, tspeed, 0, 1)
    time.sleep(0.7)
    # arlo.stop()
    # time.sleep(2)
    # if i < 20:
    #     arlo.go_diff(tspeed, tspeed, 0, 1)
    # elif i > 100:
    #     i = 0
    # else:
    #     arlo.stop()
    return i + 1


def turn_right():
    print("moving right")
    arlo.go_diff(tspeed_slow, tspeed_slow, 1, 0)
    time.sleep(0.5)


def cam_on(preview: bool = False):

    while True:
        im = picam2.capture_array("main")
        # print(im.shape)
        if preview:
            cv2.imshow("Camera", im)
            cv2.waitKey(1)
        return im


map_x, map_y = ([0], [0])
fig, ax = plt.subplots()

img = None


def update(frame):
    plt.draw()
    plt.xscale("linear")
    plt.yscale("linear")
    ax.clear()  # clearing the axes
    ax.scatter(
        map_x, map_y, c="b", alpha=0.5, vmin=-1000, vmax=1000
    )  # creating new scatter chart with updated data
    fig.canvas.draw()  # forcing the artist to redraw itself



anim = FuncAnimation(fig, update)
plt.show(block=False)

i = 0
stop_and_see = 5
j = 0
while 1 and __name__ == "__main__":
    arlo.stop()
    image = cam_on()
    img = image

    (corners, ids, rejected) = detect(image)


    cX = None
    cY = None
    topRight = None
    bottomRight = None
    bottomLeft = None
    topLeft = None

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        b, a, c = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            MARKER_HEIGHT,
            CAMERA_MATRIX,
            DISTORTION_COEFFICIENT,
        )
        # print(corners)
        marker_map = ([ax for ((ax, ay, az),) in a], [az for ((ax, ay, az),) in a])
        print(np.linalg.norm(a))
        map_x += marker_map[0]
        map_y += marker_map[1]
