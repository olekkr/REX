MAX_VOLT = 12
MAX_RPM = 100
ROBOT_DIAMETER = 39.5  # cm +- 5 mm
WHEEL_DIAMETER = 15.5 # cm
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*3.14  # cm
ROBOT_CIRCUMFERENCE = 39.5*3.14  # cm
QUARTER_TURN_64 = 0.725 # sleep


from calibrate_camera_constants import FocalLength, CameraMatrix, dimensions, downscale_factor


#FOCALLENGTH = 625.0
FOCALLENGTH = FocalLength
MARKER_HEIGHT = 145.0


# ON PId
DOWNSCALE = downscale_factor
SCREEN_RESOLUTION = dimensions
CAMERA_MATRIX = CameraMatrix


PREVIEW_DOWNSCALE = 2
ENABLE_PREVIEW = 1
CAMERA_FPS = 24

