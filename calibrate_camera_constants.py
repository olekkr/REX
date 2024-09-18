import numpy as np

FocalLength = 1880

markerHeight = 50 # mm

CameraMatrix = lambda downscale: np.matrix(
    [
        [FocalLength, 0, 1920 // (2**downscale)],
        [0, FocalLength, 1080 // (2**downscale)],
        [0, 0, 1],
    ]
)

DistortionCoefficient = np.matrix(
    [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
    ]
)
