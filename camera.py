from picamera2 import Picamera2
from picamera2 import Preview
import time
from landmark import cam_on, detect

markerDist = int(input("Distance from marker: "))
markerHeight = 50  # mm

def get_marker_dim():
    image = cam_on()
    (corners, ids, rejected) = detect(image)
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for markerCorner, markerID in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
        leftHeight = topLeft[1] - bottomLeft[1]
        rightHeight = topRight[1] - bottomRight[1]
        pixels = (leftHeight + rightHeight) // 2
        print("Right height", rightHeight)
        print("Left height", leftHeight)
        print("avg height", pixels)
        x = pixels
        X = markerHeight
        Z = markerDist
        print("focal length", x*(Z/X))

picam2 = Picamera2()
full = (1920, 1080)
preview_downscale = 2
raw = {'size': full}
main = {'size': (full[0]//(2**preview_downscale),full[1]//(2**preview_downscale))}
preview_controls = {'FrameRate': 15}
preview_config = picam2.create_preview_configuration(main, raw=raw, controls=preview_controls)
capture_controls = {'FrameRate': (2, 20)}
capture_config = picam2.create_still_configuration(controls=capture_controls)
picam2.configure(preview_config)
#camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, lores={"size": (240, 135)}, display="lores")

#camera_config = picam2.create_still_configuration()
#picam2.configure(camera_config)
picam2.start_preview(Preview.QTGL)
picam2.start()

for i in range(100):
    time.sleep(2)
    inp = input("name (q for quit): ").replace(" ", "_")
    if inp.lower() == "q":
        exit()
    picam2.capture_file(f"img{i}_{inp}.jpg")

