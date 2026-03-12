from picamera2 import Picamera2, Preview
import threading
import queue
import cv2
from PIL import Image
import numpy as np
import os
from datetime import datetime

# Preview.NULL = no display window, no GPU rendering, no X11 required
# Removes ~50-100ms/frame overhead from Qt/DRM renderer
_PREVIEW_MODE = Preview.NULL


class Camera:
    def __init__(self):
        self.picam2 = Picamera2()
        self.lock = threading.Lock()
        self.running = False

        # -------------------------------------------------------
        # OPT 2: Single full-res preview config — NO mode switch needed
        # main stream = 2028x1520 (used for scan captures AND AF preview)
        # lores stream = 320x240  (used only for autofocus variance)
        # buffer_count=4 prevents stalls waiting for next frame
        # -------------------------------------------------------
        self.preview_config = self.picam2.create_preview_configuration(
            main={"size": (2028, 1520), "format": "BGR888"},
            lores={"size": (320, 240), "format": "YUV420"},
            display="main",
            buffer_count=4
        )
        self.picam2.configure(self.preview_config)

        # -------------------------------------------------------
        # OPT 3: Async write queue — SSD writes never block the scan loop
        # Producer: capture_fullres_image() puts numpy array on queue instantly
        # Consumer: _writer_loop() saves TIFF in background
        # maxsize=8 provides buffer for burst captures
        # -------------------------------------------------------
        self._write_queue = queue.Queue(maxsize=8)
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()
        self._last_saved_path = None

    # -------------------------------------------------
    # START / STOP
    # -------------------------------------------------
    def start(self):
        with self.lock:
            if not self.running:
                self.picam2.start_preview(_PREVIEW_MODE)   # NULL = headless, no renderer
                self.picam2.start()
                self.running = True

    def stop(self):
        # Drain write queue first — ensures no frames are lost before shutdown
        self._write_queue.join()
        with self.lock:
            if self.running:
                self.picam2.stop()
                self.running = False

    # -------------------------------------------------
    # LOW-RES FRAME — reads lores stream only (fast, no main stream touch)
    # -------------------------------------------------
    def capture_lowres_for_autofocus(self):
        """
        Reads from lores stream (320x240).
        Zero interference with main stream — safe to call during preview.
        """
        with self.lock:
            if not self.running:
                return None
            yuv = self.picam2.capture_array("lores")

        # Extract Y plane from YUV420 for grayscale (no color conversion needed)
        h = yuv.shape[0] * 2 // 3
        gray = yuv[:h, :]
        return cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)

    # -------------------------------------------------
    # OPT 2: FULL-RES CAPTURE — capture_array() from main stream
    # ~10-30ms vs ~350ms with switch_mode_and_capture_image
    # OPT 3: Queues write to background thread — returns path instantly
    # -------------------------------------------------
    def capture_fullres_image(self, save_dir):
        """
        Grabs 2028x1520 frame from running preview stream.
        No sensor reconfiguration — reads the latest buffer directly.
        Write to SSD is queued asynchronously — scan loop is never blocked.
        """
        with self.lock:
            if not self.running:
                return None
            image_array = self.picam2.capture_array("main")  # 2028x1520 BGR

        os.makedirs(save_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"focused_{ts}.tiff"
        save_path = os.path.join(save_dir, filename)

        # Non-blocking: writer thread handles disk I/O while scan continues
        self._write_queue.put((image_array.copy(), save_path))
        self._last_saved_path = save_path
        return save_path

    # -------------------------------------------------
    # OPT 3: BACKGROUND WRITER THREAD
    # Continuously pops (array, path) from queue and writes TIFF to SSD
    # Runs as daemon — exits automatically when main script ends
    # -------------------------------------------------
    def _writer_loop(self):
        while True:
            try:
                image_array, save_path = self._write_queue.get(timeout=5)
                Image.fromarray(image_array).save(save_path, format="TIFF")
                self._write_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Camera Writer ERROR] {e}")
                self._write_queue.task_done()
