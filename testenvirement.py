import time
import cv2
import constants
import matplotlib.pyplot as plt
import detection
import movement
import numpy

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

landmarks = []

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

    corners, ids, _ = detection.detect(image)
    imageCopy = image.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(imageCopy, corners, ids)
        tvecs = detection.estimateDistance(corners)

        # Udregner afstand (idk om det er rigtigt)
        for i in range(len(ids)):            
            x, y, z = tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2]
            # dist = numpy.sqrt(x**2 + y**2 + z**2)
            landmarks.append([x,y])
            #print("ID: ",ids[i][0], "| Distance: ", dist)

        movement.TEST_TOWARDS_TARGET(corners, last_seen)
    else:
        movement.TEST_FIND_TARGET(last_seen)

    cv2.imshow("Image", imageCopy)
cap.release()
cv2.destroyAllWindows()

# IDK HAR BARE PLOTTET AFSTANDEN MED ID
if landmarks:
    x = [landmark[0] for landmark in landmarks]
    y = [landmark[1] for landmark in landmarks]
    plt.axhline(y=0, color='k')
    plt.ayhline(x=0, color='k')
    plt.scatter(x,y)
    plt.xlabel("ID")
    plt.ylabel("Distance")
    plt.legend()
    plt.show()