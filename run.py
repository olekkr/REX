import time

import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import detection
import movement

# from matplotlib.patches import Circle
from camera.webcam import Camera
from constants import Constants

picam2 = Camera()
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()
last_seen = None
landmark_dict = {}

# ONLY FOR PLOTTING
plt.rcParams["figure.figsize"] = 4, 4
fig, ax = plt.subplots()
ax.set_xlim(-1000, 1000)
ax.set_ylim(-300, 1700)
ax.set_aspect("equal", adjustable="box")

robot_area = plt.Circle((0.0, 0, 0), Constants.Robot.RADIUS, color="r", fill=False)
ax.add_artist(robot_area)

landmark_areas = []


def update(frame):
    image = picam2.take_image()


    corners, ids, _ = detection.detect(image)
    imageCopy = image.copy()

    # YEETER ALLE LANDMARKS
    for landmark in landmark_areas:
        landmark.remove()
    landmark_areas.clear()

    collision = False
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(imageCopy, corners, ids)
        tvecs = detection.estimateDistance(corners)

        for i in range(len(ids)):
            m_x = tvecs[i][0][0]
            m_z = tvecs[i][0][2]

            landmark_obstacle = plt.Circle(
                (m_x, m_z), Constants.Obstacle.SHAPE_RADIUS, color="b", alpha=0.5
            )
            landmkar_id = ax.text(
                m_x,
                m_z,
                str(ids[i][0]),
                color="b",
                fontsize=Constants.Obstacle.SHAPE_MIN / 10,
                ha="center",
                va="center",
            )
            ax.add_artist(landmark_obstacle)
            landmark_areas.append(landmark_obstacle)
            landmark_areas.append(landmkar_id)

            if (
                detection.DISTANCES(0, m_x, 0, m_z)
                <= Constants.Obstacle.SHAPE_RADIUS + Constants.Robot.RADIUS
            ):
                collision = True
                movement.TEST_AVOID_OBSTACLE()
                break

        if not collision:
            pass
            # movement.TEST_TOWARDS_TARGET(corners, last_seen)
    else:
        pass
        # movement.TEST_FIND_TARGET(last_seen)

    return landmark_areas + [robot_area]


ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)
plt.grid()
plt.show()
