from abc import ABC, abstractmethod
from typing import Any

import cv2
import numpy as np

from constants import Constants


class CameraBase(ABC):
    @abstractmethod
    def setup_camera(self) -> Any:
        """"""

    def __init__(self):
        self.preview_name = "Image"
        self.resize_dimensions = (800, 600)
        self.interpolation = cv2.INTER_AREA

        self.cam = self.setup_camera()

    def preview(self, image):
        cv2.waitKey(1)
        cv2.imshow(
            self.preview_name,
            cv2.resize(image, self.resize_dimensions, interpolation=self.interpolation),
        )

    @abstractmethod
    def take_image(self, enable_preview=Constants.PID.ENABLE_PREVIEW) -> cv2.typing.MatLike:
        pass

    def read(self, *args, **kwargs):
        img = self.take_image(kwargs.get("enable_preview") is True)
        return bool(img), img

    def capture_array(self, *args, **kwargs) -> cv2.typing.MatLike:
        return self.take_image(kwargs.get("enable_preview") is True)
