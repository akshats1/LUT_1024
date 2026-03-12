from picamera2 import Picamera2, Preview
import multiprocessing
import threading
import numpy as np
from PIL import Image
import cv2
import os
from datetime import datetime

# No Qt window, no GPU, no X11 overhead
_PREVIEW_MODE = Preview.NULL

# Ramdisk — pure RAM, ~10GB/s vs ~100MB/s SSD
RAMDISK_DIR = "/dev/shm/scan_buffer"


# ============================================================
# WRITER PROCESS — separate CPU core, handles all SSD I/O
# Main scan loop never waits for disk
# ============================================================
def _writer_process(queue: multiprocessing.Queue, ready_event: multiprocessing.Event):
    os.makedirs(RAMDISK_DIR, exist_ok=True)
    ready_event.set()
    while True:
        item = queue.get()
        if item is None:
            break
        npy_path, tiff_path = item
        try:
            arr = np.load(npy_path)                               # ~1ms  RAM
            Image.fromarray(arr).save(tiff_path, format="TIFF")  # ~90ms SSD
            os.remove(npy_path)
        except Exception as e:
            print(f"[Writer ERROR] {e}")


class Camera:
    def __init__(self):
        self.picam2  = Picamera2()
        self.lock    = threading.Lock()
        self.running = False

        # Full-res preview — no mode switch needed ever
        #   main  = 2028x1520 BGR888  → scan captures  (~10ms vs ~350ms)
        #   lores = 320x240   YUV420  → autofocus only
        self.preview_config = self.picam2.create_preview_configuration(
            main={"size": (2028, 1520), "format": "BGR888"},
            lores={"size": (320, 240),  "format": "YUV420"},
            display="main",
            buffer_count=4
        )
        self.picam2.configure(self.preview_config)

        os.makedirs(RAMDISK_DIR, exist_ok=True)
        self._mp_queue    = multiprocessing.Queue(maxsize=32)
        self._ready       = multiprocessing.Event()
        self._writer_proc = multiprocessing.Process(
            target=_writer_process,
            args=(self._mp_queue, self._ready),
            daemon=True,
            name="SSD-Writer"
        )
        self._writer_proc.start()
        self._ready.wait(timeout=5)
        print(f"[Camera] Writer process ready (PID={self._writer_proc.pid})")
        self._frame_counter = 0

    def start(self):
        with self.lock:
            if not self.running:
                self.picam2.start_preview(_PREVIEW_MODE)
                self.picam2.start()
                self.running = True

    def stop(self):
        # Drain all pending writes before shutdown — no frames lost
        self._mp_queue.put(None)
        self._writer_proc.join(timeout=60)
        if self._writer_proc.is_alive():
            print("[Camera] WARNING: writer did not finish — terminating")
            self._writer_proc.terminate()
        try:
            for f in os.listdir(RAMDISK_DIR):
                os.remove(os.path.join(RAMDISK_DIR, f))
        except Exception:
            pass
        with self.lock:
            if self.running:
                self.picam2.stop()
                self.running = False

    def capture_lowres_for_autofocus(self):
        """Reads lores stream (320x240) — never touches main stream."""
        with self.lock:
            if not self.running:
                return None
            yuv = self.picam2.capture_array("lores")
        h    = yuv.shape[0] * 2 // 3
        gray = yuv[:h, :]
        return cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)

    def capture_fullres_image_to(self, filepath):
        """
        Capture 2028x1520 to explicit filepath.
        Used by scan loop for serpentine-named files.
        Non-blocking — SSD write runs on writer process.
        """
        with self.lock:
            if not self.running:
                return None
            image_array = self.picam2.capture_array("main")   # ~10-30ms

        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        self._frame_counter += 1
        npy_path = os.path.join(RAMDISK_DIR, f"frame_{self._frame_counter:06d}.npy")
        np.save(npy_path, image_array)            # ~1ms to RAM
        self._mp_queue.put((npy_path, filepath))  # hand off, non-blocking
        return filepath

    def capture_fullres_image(self, save_dir):
        """Auto-named timestamped file inside save_dir. Legacy helper."""
        self._frame_counter += 1
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filepath = os.path.join(save_dir, f"focused_{ts}_{self._frame_counter:04d}.tiff")
        os.makedirs(save_dir, exist_ok=True)
        return self.capture_fullres_image_to(filepath)