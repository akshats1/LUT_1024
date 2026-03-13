from picamera2 import Picamera2, Preview
import multiprocessing
import threading
import numpy as np
from PIL import Image
import cv2
import os
from datetime import datetime

_PREVIEW_MODE = Preview.NULL

# Ramdisk — pure RAM, ~10GB/s vs ~100MB/s SSD
RAMDISK_DIR = "/dev/shm/scan_buffer"

# ============================================================
# WRITER PROCESS — separate CPU core, handles all SSD I/O
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
            arr = np.load(npy_path)
            Image.fromarray(arr).save(tiff_path, format="TIFF")
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

        # FIX 1: Queue grown from 32 → 256 to prevent backpressure blocking
        # At 90ms/write a 256-slot queue buffers ~23s of burst — more than enough
        self._mp_queue    = multiprocessing.Queue(maxsize=256)
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

    # FIX 2: Lock AEC/AGC before scan so sensor stops adapting mid-frame.
    # Rolling shutter skew is caused by the ISP adjusting exposure between
    # rows. Locking ExposureTime + AnalogueGain freezes the ISP pipeline.
    def lock_exposure_for_scan(self):
        """
        Call this ONCE after camera.start() and before the scan loop.
        Captures a few frames to let AEC/AGC converge, then freezes the
        exposure and gain. This eliminates inter-row ISP adjustments that
        cause rolling shutter skew on the Pi camera.
        """
        import time
        print("[Camera] Letting AEC/AGC converge (2s)...")
        time.sleep(2.0)  # let it settle in auto mode first

        # Read the current converged values from metadata
        with self.lock:
            metadata = self.picam2.capture_metadata()

        exp  = metadata.get("ExposureTime", 5000)
        gain = metadata.get("AnalogueGain", 1.0)

        # Now lock them — no more ISP adjustments during scan
        self.picam2.set_controls({
            "AeEnable":         False,
            "ExposureTime":     exp,
            "AnalogueGain":     gain,
        })
        time.sleep(0.3)  # one frame for controls to take effect
        print(f"[Camera] Exposure locked: ExposureTime={exp}us  Gain={gain:.2f}")

    def unlock_exposure(self):
        """Re-enable AEC/AGC after scan if needed."""
        self.picam2.set_controls({"AeEnable": True})

    def stop(self):
        # Drain all pending writes before shutdown — no frames lost
        self._mp_queue.put(None)
        self._writer_proc.join(timeout=120)
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
        Non-blocking — SSD write runs on writer process.
        Returns (filepath, queue_depth) so caller can monitor backpressure.
        """
        with self.lock:
            if not self.running:
                return None, 0
            image_array = self.picam2.capture_array("main")   # ~10-30ms

        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        self._frame_counter += 1
        npy_path = os.path.join(RAMDISK_DIR, f"frame_{self._frame_counter:06d}.npy")
        np.save(npy_path, image_array)
        self._mp_queue.put((npy_path, filepath))
        return filepath, self._mp_queue.qsize()

    def capture_fullres_image(self, save_dir):
        """Auto-named timestamped file inside save_dir. Legacy helper."""
        self._frame_counter += 1
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filepath = os.path.join(save_dir, f"focused_{ts}_{self._frame_counter:04d}.tiff")
        os.makedirs(save_dir, exist_ok=True)
        path, _ = self.capture_fullres_image_to(filepath)
        return path

    @property
    def queue_depth(self):
        return self._mp_queue.qsize()