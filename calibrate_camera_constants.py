import numpy as np

downscale_factor = 2

dimensions = (1920 // 2**downscale_factor, 1080 // 2**downscale_factor)

# FocalLength = 620.0
FocalLengthArr = [2540, 1270, 625.0]
FocalLength = FocalLengthArr[downscale_factor] if len(FocalLengthArr) <= downscale_factor else FocalLengthArr[0] / downscale_factor

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
