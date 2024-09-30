from abc import ABC, abstractmethod
from typing import Any, Optional

import cv2
import numpy as np

import custom_types
from constants import Constants


class CameraBase(ABC):
    @abstractmethod
    def setup_camera(self) -> Any:
        pass

    def __init__(
        self,
        preview_name: str = "Image",
        resize_dimensions: tuple[int, int] = (800, 600),
        interpolation: int = cv2.INTER_AREA,
        FPS: int = Constants.PID.CAMERA_FPS,
        video_configuration: Optional[dict[str, Any]] = None,
        still_configuration: Optional[dict[str, Any]] = None,
        preview_configuration: Optional[dict[str, Any]] = None,
        picam_show_preview: bool = False,
    ):
        self.preview_name = preview_name
        self.resize_dimensions = resize_dimensions
        self.interpolation = interpolation
        self.FPS = FPS
        self.frame_duration_limit = int(1 / FPS * 1000000)  # Microseconds
        if video_configuration is None:
            self.video_configuration = {
                "main": {"size": (1640, 1232), "format": "RGB888"},
                "controls": {
                    "ScalerCrop": (0, 0, 1640, 1232),
                    "FrameDurationLimits": (self.frame_duration_limit, self.frame_duration_limit),
                },
                "queue": False,
            }
        else:
            self.video_configuration = video_configuration

        self.still_configuration = still_configuration
        self.preview_configuration = preview_configuration
        self.picam_show_preview = picam_show_preview

        self.cam = self.setup_camera()

    def preview(self, image):
        if image is not None:
            cv2.waitKey(1)
            cv2.imshow(
                self.preview_name,
                cv2.resize(image, self.resize_dimensions, interpolation=self.interpolation),
            )

    @abstractmethod
    def take_image(self, enable_preview=Constants.PID.ENABLE_PREVIEW) -> custom_types.MatLikeT:
        pass

    def read(self, *args, **kwargs):
        img = self.take_image(kwargs.get("enable_preview") is True)
        return bool(img), img

    def capture_array(self, *args, **kwargs) -> custom_types.MatLikeT:
        return self.take_image(kwargs.get("enable_preview") is True)

    @abstractmethod
    def capture_file(self, name: str):
        pass
