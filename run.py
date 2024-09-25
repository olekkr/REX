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

imageSize = (constants.SCREEN_RESOLUTION)
center_image = (imageSize[0] // 2, imageSize[1] // 2)
FPS = 5
frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds

picam2 = Picamera2()
picam2_config = picam2.create_video_configuration(
    {"size": imageSize, "format": "RGB888"},
    controls={"ScalerCrop": {0, 0, 1280, 720},"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)}, queue=False,
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
ax.set_ylim(-300, 1700)
ax.set_aspect('equal', adjustable='box')

robot_area = plt.Circle((0.0, 0,0), constants.ROBOT_RADIUS, color="r", fill=False)
ax.add_artist(robot_area)

landmark_areas = []

def update(frame):

    image = picam2.capture_array("main")
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
            
            landmark_obstacle = plt.Circle((m_x, m_z), constants.OBSITCLE_SHAPE_RADIUS, color="b", alpha=0.5)
            landmkar_id = ax.text(m_x, m_z, str(ids[i][0]), color='b', fontsize=constants.OBSITCLE_SHAPE_MIN/10, ha='center', va='center')
            ax.add_artist(landmark_obstacle)
            landmark_areas.append(landmark_obstacle)
            landmark_areas.append(landmkar_id)

            if detection.DISTANCES(0, m_x, 0, m_z) <= constants.OBSITCLE_SHAPE_RADIUS + constants.ROBOT_RADIUS:
                collision = True
                movement.TEST_AVOID_OBSTACLE()
                break
        
        if not collision: 
            pass
            #movement.TEST_TOWARDS_TARGET(corners, last_seen)
    else:
        pass
        #movement.TEST_FIND_TARGET(last_seen)

    if constants.ENABLE_PREVIEW:
        cv2.imshow("Image", imageCopy)

    return landmark_areas + [robot_area]

ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)
plt.grid()
plt.show()