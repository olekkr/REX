import time

import cv2
from picamera2 import Picamera2

import robot

arlo = robot.Robot()

speed = 40
tspeed = 32
aruco = False
image = "aruco.png"
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

imageSize = (1280, 720)
FPS = 30
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
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image=image_inp,
        dictionary=arucoDict,
        parameters=arucoParams
    )
    return corners, ids, rejected

def drive_straight():
    print("Found target")
    # arlo.go_diff(speed, speed, 1, 1)


def turn_left():
    arlo.go_diff(tspeed, tspeed, 0, 1)


def turn_right():
    arlo.go_diff(tspeed, tspeed, 1, 0)


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
        im = picam2.capture_array()

        grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        print(detect(grey))

        cv2.imshow("Camera", im)
        cv2.waitKey(1)




while 1:
    cam_on()
    (corners, ids, rejected) = cv2.auro.detectMarkers(arucoDict)
    front = arlo.read_sensor(0)
    lefts = arlo.read_sensor(2)
    rights = arlo.read_sensor(3)

    qr_leftdown = corners[0][0]
    qr_rightdown = corners[0][1]

    # TESTING
    print("LD: ", qr_leftdown)
    print("RD: ", qr_rightdown)

    qr_leftUp = corners[1][0]
    qr_rightUp = corners[1][1]
    print("LU: ", qr_leftUp)
    print("RU: ", qr_rightUp)

    middle = (qr_leftdown + qr_rightdown) / 2
    close_middle = [middle - 50, middle + 50]

    if middle > close_middle[0] and middle < close_middle[1]:
        drive_straight()
    elif qr_leftdown < middle and qr_rightdown < middle:
        turn_right()
    else:
        turn_left()
