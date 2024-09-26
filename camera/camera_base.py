from abc import ABC, abstractmethod

import cv2

from constants import Constants


class Camera(ABC):
    @abstractmethod
    def setup_camera(self):
        pass

    def __init__(self):
        self.preview_name = "Image"
        self.resize_dimensions = (800, 600)
        self.interpolation = cv2.INTER_AREA

        self.setup_camera()

    def take_image(self):
        image = self.cam.capture_array("main")

        if constants.ENABLE_PREVIEW:
            cv2.waitKey(1)
            cv2.imshow(
                self.preview_name,
                cv2.resize(image, self.resize_dimensions, interpolation=self.interpolation),
            )
    
    def read()
