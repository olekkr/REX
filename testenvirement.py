import time
import cv2
import constants
import matplotlib.pyplot as plt
import detection
import movement
import numpy as np
plt.rcParams["figure.figsize"] = 4,3
from calibrate_camera_constants import CameraMatrix, DistortionCoefficient, markerHeight
from matplotlib.animation import FuncAnimation

""""
from picamera2 import Picamera2, Preview
preview_downscale = 2
imageSize = (1920 // 2**preview_downscale, 1080 // 2**preview_downscale)
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = 5
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
"""

# KUN TIL MIT CAMERA MACBOOK AIR
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("camera not found")
    exit()

speed = constants.HALF_SPEED
turning_speed = constants.MIN_SPEED
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()
last_seen = None

map_x, map_y = ([], [])
fig, ax = plt.subplots()
img = None

def update(frame):
    # if img:
    #     cv2.imshow("Image", img)
    plt.draw()
    ax.clear()
    ax.scatter(map_x, map_y, c="w", alpha=0.5, vmin=-1000, vmax=1000, s=constants.OBSITCLE_SHAPE_MAX, edgecolors="r")
    fig.canvas.draw()    
anim = FuncAnimation(fig, update)

diction = {}

scatter_plot = plt.scatter([], [], c="b")
def update_plot(m_x, m_z):
    scatter_plot.set_offsets(list(zip(m_x, m_z)))  # Update the x, z points
    plt.draw()
    plt.pause(0.7)  # Pause to show the update


plt.ion()


def distance(x, xi, y, yi, ri, r):
    if np.sqrt((x - xi) ** 2 + (y - yi) ** 2) <= ri + r:
        return True
    else:
        return False

while 1 and __name__ == "__main__":
    if cv2.waitKey(1) == ord('q'):
        break
    """"
    ret, image = picam2.capture_array()
    """
    ret, image = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # # Only for plotting
    # ax.scatter(0, 0, c="r", s=constants.OBSITCLE_SHAPE_MAX, edgecolors="b")
    # ax.set_xlim(-400, 400)
    # ax.set_ylim(-10, 1000)

    corners, ids, _ = detection.detect(image)
    imageCopy = image.copy()

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(imageCopy, corners, ids)
        tvecs = detection.estimateDistance(corners)
        # Udregner afstand (idk om det er rigtigt)

        for i in range(len(ids)):
            marker_id = ids[i][0]
            x_pos = tvecs[i][0][0]
            z_pos = tvecs[i][0][2]
            diction[marker_id] = (x_pos, z_pos)
            print(f"ID: {marker_id} - x: {x_pos} - z: {z_pos}")
            # m_id, m_x, m_z = (i for i in ids), [x for ((x, y, z),) in tvecs], [z for ((x, y, z),) in tvecs]
            # diction[m_id] = (m_x, m_z)
            if distance(0, x_pos, 0, z_pos, constants.OBSITCLE_SHAPE_MAX, constants.ROBOT_RADIUS):
                movement.TEST_AVOID_OBSTACLE()
                # movement.TEST_AVOID_OBSTACLE()


        # for marker_id, (x, z) in diction.items():
        #     ax.scatter(x, z, c="b", s=constants.OBSITCLE_SHAPE_MAX, edgecolors="r")
        # plt.draw()
        # plt.pause(0.01)

            # map_x.append(marker_map[0])
            # map_y.append(marker_map[1])
            # map_x.append([0.0])
            # map_y.append([0.0])
            # plt.scatter(0.0, 0.0, c="b")
            # plt.scatter(map_x, map_y)
            # plt.pause(0.7)
        # movement.TEST_TOWARDS_TARGET(corners, last_seen)
    
    else:
        # ax.clear()
        # ax.set_xlim(-400, 400)
        # ax.set_ylim(0, 1000)
        movement.TEST_FIND_TARGET(last_seen)

    # # Show preview but is not needed cuz it is laggy
    #cv2.imshow("Image", imageCopy)



# ani = FuncAnimation(fig, update, interval=2000)
# plt.show()


cap.release()
cv2.destroyAllWindows()
plt.close()


# # # IDK HAR BARE PLOTTET AFSTANDEN MED ID
# # if landmarks:
# #     x = [landmark[0] for landmark in landmarks]
# #     y = [landmark[1] for landmark in landmarks]
# #     # plt.axhline(x=0)
# #     # plt.axvline(y=0, color='k')
# #     plt.scatter(x,y)
# #     plt.xlabel("x")
# #     plt.ylabel("y")
# #     plt.legend()
# #     plt.show()