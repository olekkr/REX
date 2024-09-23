import queue
import threading
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from picamera2 import Picamera2

import robot
from calibrate_camera_constants import CameraMatrix, DistortionCoefficient, markerHeight

arlo = robot.Robot()
data_queue = queue.Queue()
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


picam2 = Picamera2()
picam2_config = picam2.create_video_configuration(
    {"size": imageSize, "format": "RGB888"},
    controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
    queue=False,
)
picam2.configure(picam2_config)  # Not really necessary
picam2.start(show_preview=False)
time.sleep(2)
picam2.start()
# cap = cv2.VideoCapture()

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


def preview():

    try:
        queued_img = data_queue.get(timeout=0.1)
        if queued_img:
            cv2.imshow("Camera", queued_img)
            cv2.waitKey(1)
    except queue.Empty:
        return


def cam_on():
    while True:
        im = picam2.capture_array("main")

        return im


map_x, map_y = ([0], [0])
fig, ax = plt.subplots()

img = None

preview_thread = threading.Thread(target=preview)


def update(frame):
    plt.draw()
    ax.clear()  # clearing the axes
    ax.scatter(
        map_x,
        map_y,
        c="b",
        alpha=0.5,
    )  # creating new scatter chart with updated data
    fig.canvas.draw()  # forcing the artist to redraw itself


anim = FuncAnimation(fig, update)
plt.show(block=False)
preview_thread.start()

i = 0
stop_and_see = 5
j = 0
while 1 and __name__ == "__main__":
    # arlo.stop()
    image = cam_on()
    img = image
    data_queue.put(img)
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
            markerHeight,
            CameraMatrix,
            DistortionCoefficient,
        )
        # print(corners)
        marker_map = ([ax for ((ax, ay, az),) in a], [az for ((ax, ay, az),) in a])
        # print(np.linalg.norm(a))
        map_x += marker_map[0]
        map_y += marker_map[1]
preview_thread.join()
