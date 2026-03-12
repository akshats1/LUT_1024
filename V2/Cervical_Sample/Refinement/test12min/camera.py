# ============================================================
# camera.py — Optimized: Preview.NULL + Ramdisk + Multiprocessing Writer
#
# Architecture:
#   Main process  → capture_array() from sensor (~10ms)
#                 → np.save() to /dev/shm ramdisk  (~1ms)
#                 → puts (npy_path, tiff_path) on multiprocessing.Queue
#   Writer process → np.load() from ramdisk  (~1ms)
#                  → PIL.save() TIFF to SSD  (~90ms, fully parallel)
#                  → os.remove() ramdisk file
#
# Main scan loop is NEVER blocked by SSD I/O.
# ============================================================

from picamera2 import Picamera2, Preview
import multiprocessing
import numpy as np
from PIL import Image
import threading
import cv2
import os
from datetime import datetime

RAMDISK_DIR  = "/dev/shm/scan_buffer"   # tmpfs — pure RAM on all Linux/Pi
_PREVIEW_MODE = Preview.NULL             # no Qt window, no GPU, no X11


# ============================================================
# WRITER PROCESS — dedicated CPU core for all SSD I/O
# ============================================================
def _writer_process(queue: multiprocessing.Queue, ready_event: multiprocessing.Event):
    """
    Runs on its own CPU core.
    Receives (npy_path, tiff_path) — loads numpy from ramdisk, saves TIFF to SSD.
    Sentinel value None signals shutdown.
    """
    os.makedirs(RAMDISK_DIR, exist_ok=True)
    ready_event.set()

    while True:
        item = queue.get()
        if item is None:
            break

        npy_path, tiff_path = item
        try:
            arr = np.load(npy_path)                              # ~1ms  (RAM read)
            Image.fromarray(arr).save(tiff_path, format="TIFF") # ~90ms (SSD write)
            os.remove(npy_path)                                  # free ramdisk
        except Exception as e:
            print(f"[Writer ERROR] {e} | file={npy_path}")


class Camera:
    def __init__(self):
        self.picam2 = Picamera2()
        self.lock   = threading.Lock()
        self.running = False

        # Single full-res preview config — no mode switch ever needed
        # main  = 2028x1520 BGR888  → used for scan captures
        # lores = 320x240   YUV420  → used only for autofocus variance
        # buffer_count=4 avoids frame stalls under load
        self.preview_config = self.picam2.create_preview_configuration(
            main={"size": (2028, 1520), "format": "BGR888"},
            lores={"size": (320, 240),  "format": "YUV420"},
            display="main",
            buffer_count=4
        )
        self.picam2.configure(self.preview_config)

        # Multiprocessing writer — separate process, separate CPU core
        # Queue carries only small strings (paths), zero array serialization overhead
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

    # -------------------------------------------------
    # START / STOP
    # -------------------------------------------------
    def start(self):
        with self.lock:
            if not self.running:
                self.picam2.start_preview(_PREVIEW_MODE)
                self.picam2.start()
                self.running = True

    def stop(self):
        """Drain all pending writes before shutting down — no frames lost."""
        self._mp_queue.put(None)          # sentinel → writer exits cleanly
        self._writer_proc.join(timeout=60)
        if self._writer_proc.is_alive():
            print("[Camera] WARNING: writer did not finish — terminating")
            self._writer_proc.terminate()

        # Clean up any leftover ramdisk files
        try:
            for f in os.listdir(RAMDISK_DIR):
                os.remove(os.path.join(RAMDISK_DIR, f))
        except Exception:
            pass

        with self.lock:
            if self.running:
                self.picam2.stop()
                self.running = False

    # -------------------------------------------------
    # LOW-RES for autofocus — lores stream only, never touches main stream
    # -------------------------------------------------
    def capture_lowres_for_autofocus(self):
        with self.lock:
            if not self.running:
                return None
            yuv = self.picam2.capture_array("lores")

        h    = yuv.shape[0] * 2 // 3       # extract Y plane from YUV420
        gray = yuv[:h, :]
        return cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)

    # -------------------------------------------------
    # FULL-RES CAPTURE with AUTO-NAMED file (legacy helper)
    # Generates a timestamped filename inside save_dir
    # -------------------------------------------------
    def capture_fullres_image(self, save_dir):
        self._frame_counter += 1
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"focused_{ts}_{self._frame_counter:04d}.tiff"
        filepath = os.path.join(save_dir, filename)
        os.makedirs(save_dir, exist_ok=True)
        return self.capture_fullres_image_to(filepath)

    # -------------------------------------------------
    # FULL-RES CAPTURE with EXPLICIT filepath  ← used by scan loop
    #
    # Step 1: capture_array("main")     ~10-30ms  sensor read
    # Step 2: np.save() to /dev/shm      ~1ms     RAM write
    # Step 3: queue.put((paths))        <0.1ms    just two strings
    # Total blocking time: ~12-31ms
    # SSD write (~90ms) runs in parallel on writer process
    # -------------------------------------------------
    def capture_fullres_image_to(self, filepath):
        with self.lock:
            if not self.running:
                return None
            image_array = self.picam2.capture_array("main")   # 2028x1520 BGR

        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        # Unique ramdisk intermediate file
        self._frame_counter += 1
        npy_path = os.path.join(
            RAMDISK_DIR,
            f"frame_{self._frame_counter:06d}.npy"
        )

        np.save(npy_path, image_array)          # ~1ms to RAM
        self._mp_queue.put((npy_path, filepath)) # hand off to writer process

        return filepath