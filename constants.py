import numpy as np


class Constants:
    class Robot:
        MAX_VOLT = 12
        MAX_RPM = 100
        DIAMETER = 395  # mm +- 5 mm
        RADIUS = DIAMETER / 2
        CIRCUMFERENCE = 39.5 * 3.14  # cm
        SHAPE = (50, 50)  # Around 50 cm before the landmarks are visible in the camera
        WHEEL_DIAMETER = 155  # mm
        WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14  # cm
        QUARTER_TURN_64 = 0.725  # sleep

    class Sensor:
        MAX_SPEED = 100
        HALF_SPEED = MAX_SPEED / 2
        MIN_SPEED = 30
        THRESHOLD = 100
        FINDING_SLEEP = 0.5
        FRONT = 0
        BACK = 1
        LEFT = 2
        RIGHT = 3

    class Obstacle:
        SHAPE = [145, 145]  # in mm
        SHAPE_MIN = 220  # in mm
        SHAPE_MAX = 250  # in mm
        SHAPE_RADIUS = SHAPE_MAX / 2

    class PID:
        DOWNSCALE = 1
        SCREEN_RESOLUTION = (int(3280 // 2**DOWNSCALE), int(2464 // 2**DOWNSCALE))
        FOCALLENGTH_ARR = [2540, 1270, 625.0]
        FOCALLENGTH = (
            FOCALLENGTH_ARR[DOWNSCALE]
            if len(FOCALLENGTH_ARR) > DOWNSCALE
            else FOCALLENGTH_ARR[0] / 2**DOWNSCALE
        )
        MARKER_HEIGHT = 145.0
        DISTORTION_COEFFICIENT = np.array([0, 0, 0, 0, 0], dtype=float)
        CAMERA_MATRIX = np.array(
            [
                [FOCALLENGTH, 0, SCREEN_RESOLUTION[0] / 2],
                [0, FOCALLENGTH, SCREEN_RESOLUTION[1] / 2],
                [0, 0, 1],
            ],
            dtype=float,
        )
        PREVIEW_DOWNSCALE = 2
        ENABLE_PREVIEW = 1
        CAMERA_FPS = 24

    class PyPlot:
        valid_interactive_backends = [  # I think Cairo is blocking and agg is not
            "GTK3Cairo",
            "QtAgg",
            "QtCairo",
            "Qt5Agg",
            "Qt5Cairo",
        ]
        interactive_backend = "Qt5Agg"
