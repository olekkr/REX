import numpy as np

FocalLength = 648.0

markerHeight = 145.0  # mm

# 94 * (1000 / 145) = 648.2758620689655
# 1880 / (1000/50) = 94

CameraMatrix = lambda downscale: np.array(
    [
        [FocalLength, 0.0, 1920 / (2**downscale)],
        [0.0, FocalLength, 1080.0 // (2**downscale)],
        [0, 0, 1],
    ],
    dtype=float,
)

DistortionCoefficient = np.array([0, 0, 0, 0, 0], dtype=float)
