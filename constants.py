import numpy as np
from calibrate_camera_constants import FocalLength, CameraMatrix, dimensions, downscale_factor

MAX_VOLT = 12
MAX_RPM = 100
ROBOT_DIAMETER = 395  # mm +- 5 mm
WHEEL_DIAMETER = 155 # mm
ROBOT_RADIUS = ROBOT_DIAMETER/2
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*3.14  # cm
ROBOT_CIRCUMFERENCE = 39.5*3.14  # cm
QUARTER_TURN_64 = 0.725 # sleep
ROBOT_SHAPE = (50, 50)  # Arund 50 cm before the landmarks are visible in the camera

# Constants for the sensors
MAX_SPEED = 100
HALF_SPEED = MAX_SPEED/2
MIN_SPEED = 30
THRESHOLD = 100
FINDING_SLEEP = 0.5

OBSITCLE_SHAPE = [145, 145] # in mm
FRONT_SENSOR = 0
BACK_SENSOR  = 1
LEFT_SENSOR  = 2
RIGHT_SENSOR = 3

OBSITCLE_SHAPE_MIN = 220 # in mm
OBSITCLE_SHAPE_MAX = 250 # in mm
OBSITCLE_SHAPE_RADIUS = OBSITCLE_SHAPE_MAX / 2
OBSITCLE_SHAPE = OBSITCLE_SHAPE_MAX


#FOCALLENGTH = 625.0
FOCALLENGTH = FocalLength
MARKER_HEIGHT = 145.0
DISTORTION_COEFFICIENT = np.array([0, 0, 0, 0, 0], dtype=float)


# ON PId
DOWNSCALE = downscale_factor
SCREEN_RESOLUTION = dimensions
CAMERA_MATRIX = CameraMatrix

ENABLE_PREVIEW = 0
CAMERA_FPS = 24

# # FOR TESTING
# SCREEN_RESOLUTION = [1280, 720]
# CAMERA_MATRIX = np.array([
#         [FOCALLENGTH, 0.0, 1280.0/2],
#         [0.0, FOCALLENGTH, 720.0/2],
#         [0.0, 0.0, 1.0],
#     ],
#     dtype=float,
# )