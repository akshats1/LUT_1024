from picamera2 import Picamera2, Preview
import threading
import cv2
from PIL import Image
import numpy as np
import os
from datetime import datetime


class Camera:
    def __init__(self):
        self.picam2 = Picamera2()
        self.lock = threading.Lock()
        self.running = False

        # Preview configuration (continuous stream)
        self.preview_config = self.picam2.create_preview_configuration(
            main={"size": (640, 480)},
            lores={"size": (320, 240)},
            display="main"
        )
        self.picam2.configure(self.preview_config)

        # Full-resolution capture (used ONLY once at end)
        self.highres_config = self.picam2.create_still_configuration(
            main={"size": (4056, 3040)},
            raw={"format": "SBGGR10_CSI2P"}
        )

    # -------------------------------------------------
    # START / STOP CAMERA + LIVE PREVIEW
    # -------------------------------------------------
    def start(self):
        with self.lock:
            if not self.running:
                self.picam2.start_preview(Preview.QTGL)
                # For kiosk / no-X use:
                # self.picam2.start_preview(Preview.DRM)

                self.picam2.start()
                self.running = True

    def stop(self):
        with self.lock:
            if self.running:
                self.picam2.stop_preview()
                self.picam2.stop()
                self.running = False

    # -------------------------------------------------
    # LOW-RES FRAME FROM LIVE PREVIEW (SAFE FOR AF)
    # -------------------------------------------------
    def capture_lowres_for_autofocus(self):
        """
        Capture frame from running preview stream.
        DOES NOT stop preview.
        """
        with self.lock:
            if not self.running:
                return None

            frame = self.picam2.capture_array()
            return cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)

    # -------------------------------------------------
    # FINAL IMAGE (FULL RES + TIMESTAMP)
    # -------------------------------------------------
    def capture_fullres_image(self, save_dir):
        with self.lock:
            if not self.running:
                return None

            image_array = self.picam2.switch_mode_and_capture_image(
                self.highres_config, "main"
            )

            if isinstance(image_array, Image.Image):
                image_array = np.array(image_array)

            os.makedirs(save_dir, exist_ok=True)

            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"focused_{ts}.tiff"
            save_path = os.path.join(save_dir, filename)

            Image.fromarray(image_array).save(save_path, format="TIFF")
            return save_path

