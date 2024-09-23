import numpy as np

dimensions = (480, 270)

# FocalLength = 620.0
FocalLength = 625.0

markerHeight = 145.0  # mm


CameraMatrix = np.array(
    [
        [FocalLength, 0, dimensions[0] / 2],
        [0, FocalLength, dimensions[1] / 2],
        [0, 0, 1],
    ],
    dtype=float,
)
DistortionCoefficient = np.array([0, 0, 0, 0, 0], dtype=float)
