import cv2
import numpy
from constants import MARKER_HEIGHT, CAMERA_MATRIX, DISTORTION_COEFFICIENT


arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoParams = cv2.aruco.DetectorParameters()

def detect(image):
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image=image, dictionary=arucoDict)
    return corners, ids, rejected

def estimateDistance(corners, marker_height=MARKER_HEIGHT, camera_matrix=CAMERA_MATRIX, distortion_coefficient=DISTORTION_COEFFICIENT):
    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, 
            marker_height,
            camera_matrix, 
            distortion_coefficient
        )
    return tvecs

def TOP_LEFT_CORNER(corners):
    return corners[0][0][3]

def TOP_RIGHT_CORNER(corners):
    return corners[0][0][2]

def BOTTOM_RIGHT_CORNER(corners):
    return corners[0][0][1]

def BOTTOM_LEFT_CORNER(corners):
    return corners[0][0][0]

def CENTER(x1, x2):
    return (x1 + x2) / 2

def DISTANCES(x, xi, y, yi):
    return numpy.sqrt((x - xi) ** 2 + (y - yi) ** 2)
