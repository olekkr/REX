from camera.webcam import Camera
import cv2

picam2 = Camera()
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()



def preview_marker(corners, im):
    if corners:
        try:
            for corner in corners:
                picam2.preview(cv2.aruco.drawDetectedCornersCharuco(im, corner))
        except:
            pass
    else:
        picam2.preview(im)

def get_marker_dim(markerDist: int, markerHeight: int = 40, calc_f: bool = False):
    image = picam2.take_image(enable_preview=False)
    s = ""
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=image, dictionary=arucoDict)
    preview_marker(corners, image)
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

        # draw the bounding box of the ArUCo detection
        cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the image
        cv2.putText(
            image,
            str(markerID),
            (topLeft[0], topLeft[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

        leftHeight = bottomLeft[1] - topLeft[1]
        rightHeight = bottomRight[1] - topRight[1]
        heightPixels = (leftHeight + rightHeight) // 2
        topWidth = topRight[0] - topLeft[0]
        bottomWidth = bottomRight[0] - bottomLeft[0]
        heightPixels = (leftHeight + rightHeight) // 2
        widthPixels = (topWidth + bottomWidth) // 2
        # print("Right height", rightHeight)
        # print("Left height", leftHeight)
        # print("avg height", pixels)
        y = heightPixels
        Y = markerHeight
        x = widthPixels
        X = markerHeight
        Z = markerDist
        print()
        if calc_f:
            fx = x * (Z / X)
            fy = y * (Z / Y)
            s += ",".join([str(i) for i in [x, X, y, Y, Z, fx, fy]])
            print(y, Y, Z, fy)
        else:
            if markerDist:
                s += ",".join([str(i) for i in [x, X, y, Y, Z, " ", " "]])
            else:
                s += ",".join([str(i) for i in [x, X, y, Y, " ", " ", " "]])
    return s



distance = 205
while 1:
    get_marker_dim(distance, calc_f=True)
    inp = input("distance")
    if inp:
        distance = int(inp)

