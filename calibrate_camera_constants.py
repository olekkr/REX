import numpy as np

FocalLength = 1880.0

markerHeight = 50.0  # mm

CameraMatrix = lambda downscale: np.array(
    [
        [FocalLength, 0.0, 1920 / (2**downscale)],
        [0.0, FocalLength, 1080.0 // (2**downscale)],
        [0, 0, 1],
    ],
    dtype=float,
)

DistortionCoefficient = np.array([0, 0, 0, 0, 0], dtype=float)
