import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import constants
import detection
import movement

""""
from picamera2 import Picamera2, Preview
import robot

arlo = robot.Robot()

preview_downscale = 2
imageSize = (1280 // 2**preview_downscale, 720 // 2**preview_downscale)
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = 5
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds

picam2 = Picamera2()
picam2_config = picam2.create_video_configuration(
    {"size": imageSize, "format": "RGB888"},
    controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)}, queue=False,
    )
picam2.configure(picam2_config)  # Not really necessary
picam2.start(show_preview=False)

time.sleep(2)
picam2.start()
"""

# KUN TIL MIT CAMERA MACBOOK AIR
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("camera not found")
    exit()


arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()
last_seen = None
landmark_dict = {}

# while 1 and __name__ == "__main__":
#     if cv2.waitKey(1) == ord('q'):
#         break
#     """"
#     ret, image = picam2.capture_array()
#     """
#     ret, image = cap.read()
#     if not ret:
#         print("Error: Failed to capture image.")
#         break

#     corners, ids, _ = detection.detect(image)
#     imageCopy = image.copy()

#     if ids is not None:
#         cv2.aruco.drawDetectedMarkers(imageCopy, corners, ids)
#         tvecs = detection.estimateDistance(corners)

#         for i in range(len(ids)):
#             m_id, m_x, m_z = ids[i][0], tvecs[i][0][0], tvecs[i][0][2]
#             landmark_dict[m_id] = (m_x, m_z)

#             if detection.distance(0, m_x, 0, m_z, constants.OBSTACLE_SHAPE_MAX, constants.ROBOT_RADIUS):
#                 movement.TEST_AVOID_OBSTACLE()
#                 break

#             """"
#             # Only for plotting
#             plt.scatter(landmark_dict[m_id][0], landmark_dict[m_id][1], c="b")
#             plt.pause(0.01)
#             """
#         movement.TEST_TOWARDS_TARGET(corners, last_seen)
#     else:
#         movement.TEST_FIND_TARGET(last_seen)

#     # # Show preview but is not needed cuz it is laggy
#     cv2.imshow("Image", imageCopy)


# ONLY FOR PLOTTING
plt.rcParams["figure.figsize"] = 4, 4
fig, ax = plt.subplots()
ax.set_xlim(-1000, 1000)
ax.set_ylim(0, 2000)

robot_area = plt.Circle((0.0, 0.0), constants.ROBOT_RADIUS, color="r", fill=False)
ax.add_artist(robot_area)

landmark_areas = []


def update(frame):
    print("update")
    """"
    # For picamera
    image = picam2.capture_array("main")
    cv2.waitKey(1)
    """

    # For MACBOOK AIR CAMERA
    _, image = cap.read()
    cv2.waitKey(1)

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
                (m_x, m_z), constants.OBSTACLE_SHAPE_MAX, color="b", alpha=0.5
            )
            landmkar_id = ax.text(
                m_x,
                m_z,
                str(ids[i][0]),
                color="b",
                fontsize=constants.OBSTACLE_SHAPE_MIN / 10,
                ha="center",
                va="center",
            )
            ax.add_artist(landmark_obstacle)
            landmark_areas.append(landmark_obstacle)
            landmark_areas.append(landmkar_id)

            if (
                detection.DISTANCES(0, m_x, 0, m_z)
                <= constants.OBSTACLE_SHAPE_MAX + constants.ROBOT_RADIUS
            ):
                collision = True
                movement.TEST_AVOID_OBSTACLE()
                break

        if not collision:
            movement.TEST_TOWARDS_TARGET(corners, last_seen)
    else:
        movement.TEST_FIND_TARGET(last_seen)

    cv2.imshow("Image", imageCopy)

    return landmark_areas + [robot_area]


ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)
plt.show()
