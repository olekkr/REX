import cv2
import constants
import matplotlib.pyplot as plt
import detection
import movement
from matplotlib.animation import FuncAnimation
from picamera2 import Picamera2, Preview
import robot
import time

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


arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()
last_seen = None
landmark_dict = {}

# ONLY FOR PLOTTING
plt.rcParams["figure.figsize"] = 4, 4
fig, ax = plt.subplots()
ax.set_xlim(-1000, 1000)
ax.set_ylim(-50, 1950)

robot_area = plt.Circle((0.0, -constants.ROBOT_RADIUS), constants.ROBOT_RADIUS, color="r", fill=False)
ax.add_artist(robot_area)

landmark_areas = []

def update(frame):
    print("1")
    image = picam2.capture_array("main")
    cv2.waitKey(1)
    print("2")
    

    corners, ids, _ = detection.detect(image)
    print("3")
    imageCopy = image.copy()
    print("4")
    
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
            
            landmark_obstacle = plt.Circle((m_x, m_z), constants.OBSITCLE_SHAPE_MAX, color="b", alpha=0.5)
            landmkar_id = ax.text(m_x, m_z, str(ids[i][0]), color='b', fontsize=constants.OBSITCLE_SHAPE_MIN/10, ha='center', va='center')
            ax.add_artist(landmark_obstacle)
            landmark_areas.append(landmark_obstacle)
            landmark_areas.append(landmkar_id)

            if detection.DISTANCES(0, m_x, -constants.ROBOT_RADIUS, m_z) <= constants.OBSITCLE_SHAPE_MAX + constants.ROBOT_RADIUS:
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
plt.grid()
plt.show()