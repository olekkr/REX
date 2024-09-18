import robot
import cv2
from picamera2 import Picamera2
import time
arlo = robot.Robot()
cap = cv2.VideoCapture(0)

speed = 40
tspeed = 32
aruco = False
image = "aruco.png"
arucoDict = cv2.aruco.DICT_6X6_250

picam2 = Picamera2()
picam2.start()
time.sleep(2)
cap = picam2.capture()

def drive_straight():
    print("Found target")
    # arlo.go_diff(speed, speed, 1, 1)

def turn_left():
    arlo.go_diff(tspeed, tspeed, 0, 1)

def turn_right():
    arlo.go_diff(tspeed, tspeed, 1, 0)

def cam_on():
    ret, frame = cap.read()
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow('frame', rgb)

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