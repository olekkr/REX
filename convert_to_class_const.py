import fnmatch
import os
import re

old_to_new = {
    "constants.MAX_VOLT": "Constants.Robot.MAX_VOLT",
    "constants.MAX_RPM": "Constants.Robot.MAX_RPM",
    "constants.DIAMETER": "Constants.Robot.DIAMETER",
    "constants.RADIUS": "Constants.Robot.RADIUS",
    "constants.CIRCUMFERENCE": "Constants.Robot.CIRCUMFERENCE",
    "constants.SHAPE": "Constants.Robot.SHAPE",
    "constants.WHEEL_DIAMETER": "Constants.Robot.WHEEL_DIAMETER",
    "constants.WHEEL_CIRCUMFERENCE": "Constants.Robot.WHEEL_CIRCUMFERENCE",
    "constants.QUARTER_TURN_64": "Constants.Robot.QUARTER_TURN_64",
    "constants.MAX_SPEED": "Constants.Sensor.MAX_SPEED",
    "constants.HALF_SPEED": "Constants.Sensor.HALF_SPEED",
    "constants.MIN_SPEED": "Constants.Sensor.MIN_SPEED",
    "constants.THRESHOLD": "Constants.Sensor.THRESHOLD",
    "constants.FINDING_SLEEP": "Constants.Sensor.FINDING_SLEEP",
    "constants.FRONT": "Constants.Sensor.FRONT",
    "constants.BACK": "Constants.Sensor.BACK",
    "constants.LEFT": "Constants.Sensor.LEFT",
    "constants.RIGHT": "Constants.Sensor.RIGHT",
    "constants.SHAPE": "Constants.Obstacle.SHAPE",
    "constants.SHAPE_MIN": "Constants.Obstacle.SHAPE_MIN",
    "constants.SHAPE_MAX": "Constants.Obstacle.SHAPE_MAX",
    "constants.SHAPE_RADIUS": "Constants.Obstacle.SHAPE_RADIUS",
    "constants.DOWNSCALE": "Constants.PID.DOWNSCALE",
    "constants.SCREEN_RESOLUTION": "Constants.PID.SCREEN_RESOLUTION",
    "constants.FOCALLENGTH_ARR": "Constants.PID.FOCALLENGTH_ARR",
    "constants.FOCALLENGTH": "Constants.PID.FOCALLENGTH",
    "constants.MARKER_HEIGHT": "Constants.PID.MARKER_HEIGHT",
    "constants.DISTORTION_COEFFICIENT": "Constants.PID.DISTORTION_COEFFICIENT",
    "constants.CAMERA_MATRIX": "Constants.PID.CAMERA_MATRIX",
    "constants.PREVIEW_DOWNSCALE": "Constants.PID.PREVIEW_DOWNSCALE",
    "constants.ENABLE_PREVIEW": "Constants.PID.ENABLE_PREVIEW",
    "constants.CAMERA_FPS": "Constants.PID.CAMERA_FPS",
}

exclude = ["__pycache__/*", "*/__pycache__/*", ".venv/*", __file__]


def is_excluded(path):
    for pattern in exclude:
        if fnmatch.fnmatch(path, pattern):
            return True
    return False


def replace_constants_in_file(file_path):
    with open(file_path, "r+") as file:
        content = file.read()
        for old, new in old_to_new.items():
            content = re.sub(re.escape(old), new, content)
        file.seek(0)
        file.write(content)
        file.truncate()


for root, dirs, files in os.walk("."):
    dirs[:] = [d for d in dirs if not is_excluded(os.path.join(root, d))]
    for file in files:
        if not is_excluded(file):
            replace_constants_in_file(os.path.join(root, file))
