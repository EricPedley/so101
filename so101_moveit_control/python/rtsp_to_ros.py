from camera.arducam_libraries.JetsonCamera import GstCamera
from perception.camera.camera import Camera, ImageMetadata
from shared.types import Pose
from scipy.spatial.transform import Rotation
import numpy as np
from pathlib import Path
import cv2 as cv
from perception.types import Image
import time


class RTSPRosNode(Camera):

    def __init__(
        self,
        log_dir: str | Path | None = None,
        flipped=True,  # because of how they're mounted we might have to flip them sometimes.
    ):
        """
        TODO: make resolution choosable. Hard-coded in JetsonCamera rn.
        """
        super().__init__(log_dir)
        pipeline = 'rtspsrc location=rtsp://192.168.42.1/h264 latency=0 drop-on-latency=true ! rtph264depay ! h264parse ! avdec_h264 max-threads=1 ! videoconvert ! video/x-raw, format=BGRx ! appsink drop=true'

        self._camera = GstCamera(pipeline)
        self._relative_pose = Pose(
            np.array([0.1, 0, -0.05]), Rotation.from_euler("y", 90, degrees=True)
        )
        self._flipped = flipped

    def take_image(self) -> Image:
        frame = self._camera.getFrame()
        if frame is None:
            return None
        if self._flipped:
            frame = cv.rotate(frame, cv.ROTATE_180)
        return Image(frame)

    def get_metadata(self) -> ImageMetadata:
        return ImageMetadata(
            timestamp=time.time(),
            relative_pose=self._relative_pose,
            focal_len_px=self.get_focal_length_px(),
        )

    def get_focal_length_px(self):
        return 2516 * 1920 / 4056
        # Using arducam specs. 3.9mm focal length, 1.55 micrometer pixel size, 4056 pixels horizontal
        # We should actually do camera calibration at some point
