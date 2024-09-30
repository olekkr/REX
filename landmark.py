import time

import cv2
import numpy as np

import robot
from camera.picam import Camera
from constants import Constants

CAMERA_MATRIX, DISTORTION_COEFFICIENT, MARKER_HEIGHT = (
    Constants.PID.CAMERA_MATRIX,
    Constants.PID.DISTORTION_COEFFICIENT,
    Constants.PID.MARKER_HEIGHT,
)


arlo = robot.Robot()
speed = 40
tspeed_slow = 32
tspeed = 32
aruco = False
image = "aruco.png"
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
preview_downscale = 1
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


def cam_on():
    # if not cap.isOpened():
    #     print("Camera not opened")
    #     exit()
    # ret, frame = cap.read()
    # while not ret:
    #     print("Frame unavailable waiting...")
    #     time.sleep(5)
    #     ret, frame = cap.read()

    while True:
        im = picam2.capture_array("main")
        # print(im.shape)
        return im
        grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        print(detect(grey))
        cv2.imshow("Camera", im)
        cv2.waitKey(1)


i = 0
stop_and_see = 5
j = 0
while 1 and __name__ == "__main__":
    arlo.stop()
    image = cam_on()
    cv2.imshow("Image", image)
    cv2.waitKey(1)
    (corners, ids, rejected) = detect(image)

    front = arlo.read_sensor(0)
    lefts = arlo.read_sensor(2)
    rights = arlo.read_sensor(3)

    cX = None
    cY = None
    topRight = None
    bottomRight = None
    bottomLeft = None
    topLeft = None

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        a, b, c = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            MARKER_HEIGHT,
            CAMERA_MATRIX,
            DISTORTION_COEFFICIENT,
        )
        print("a", np.linalg.norm(a))
    try:
        for corner in corners:
            cv2.imshow("Image", cv2.aruco.drawDetectedCornersCharuco(image, corner))
    except Exception as e:
        print(e)
        cv2.imshow("Image", image)

    cv2.waitKey(1)
    if cv2.getWindowProperty("Image", 0) == -1:
        arlo.stop()
        exit()
        # if len(corners) == 0 and j < stop_and_see:
        #     j += 1
        #     continue
        # elif len(corners) == 0 and j >= stop_and_see:
        #     j = 0

        # # middle = (qr_leftdown + qr_rightdown) / 2
        # if cX and cY and topRight and bottomRight and bottomLeft and topLeft:
        #     imgcX, imgcY = center_image
        #     thresholdX = int(imgcX * 0.5)
        #     thresholdY = int(imgcY * 0.5)
        #     close_x = range(cX - thresholdX, cX + thresholdX)
        #     close_y = range(cY - thresholdY, cY + thresholdY)
        #     # print(close_x, close_y, cX, cY, imgcX, imgcY)

        if imgcX in close_x and imgcY in close_y:
            drive_straight(i)
        elif bottomLeft[0] > imgcX:
            turn_right()
        else:
            i = turn_left(i)
    else:
        i = turn_left(i)
