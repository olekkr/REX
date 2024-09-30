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
        DOWNSCALE = 0
        SCREEN_RESOLUTION = (1640, 1232)
        FOCALLENGTH_ARR = [2540, 1270, 625.0]
        FOCALLENGTH = 640.0
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
        PREVIEW_DOWNSCALE = 1
        PREVIEW_DIMENSIONS = (
            SCREEN_RESOLUTION[0] // (2**PREVIEW_DOWNSCALE),
            SCREEN_RESOLUTION[1] // (2**PREVIEW_DOWNSCALE),
        )
        ENABLE_PREVIEW = 1
        CAMERA_FPS = 24

    class PyPlot:
        valid_interactive_backends = [
            "GTK3Cairo",  # blocking
            "QtAgg",
            "QtCairo",
            "Qt5Agg",
            "Qt5Cairo",
        ]
        interactive_backend = "Qt5Agg"
