import robot
import cv2

arlo = robot.Robot()

cap = cv2.VideoCapture(0)

speed = 40
tspeed = 30
aruco = False
image = "aruco.png"

webResX, webResY = 1929, 1080


arucoDict = cv2.aruco_DICT_6X6_250()


def drive_straight():
    arlo.go_diff(speed, speed, 1, 1)

def turn_left():
    arlo.go_diff(tspeed, tspeed, 0, 1)

def cam_on():
    ret, frame = cap.read()
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow('frame', rgb)

while 1:
    cam_on()
    (corners, ids, rejected) = cv2.auro_detectMarkers(arucoDict)
    front = arlo.read_sensor(0)
    lefts = arlo.read_sensor(2)
    rights = arlo.read_sensor(3)

    left_corner = corners[0][0] 
    right_corner = corners[0][1]

    middle_corner = (left_corner + right_corner) / 2
    close_middle = [middle_corner - 50, middle_corner + 50]

    if middle_corner > close_middle[0] and middle_corner < close_middle[1]:
        drive_straight()
    else:
        turn_left()