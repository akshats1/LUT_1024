# ============================================
# SERPENTINE SCAN — OPTIMISED
# Target: <10 min for 32×32 (was 12.3 min)
#
# Key optimisations vs original:
#
# 1. ROLLING SHUTTER FIX — lock AEC/AGC before scan
#    camera.lock_exposure_for_scan() freezes ExposureTime + AnalogueGain
#    so the ISP never adjusts mid-frame. This is the correct fix; sleeping
#    longer is just masking the symptom.
#
# 2. PIPELINED Y MOVES — Y step issued non-blocking (wait=False) right
#    after capture. The motor moves while Python does logging + Z calc.
#    flush_move() is called at the start of the next iteration, by which
#    time the move is already done. Y move cost: ~0ms on critical path.
#
# 3. REDUCED SETTLE — 250ms → 80ms.
#    Safe because AEC/AGC is locked (no ISP ramp-up jitter) and M400
#    guarantees the planner is drained before we sleep.
#
# 4. LARGER WRITE QUEUE — 32 → 256 slots.
#    Prevents queue.put() ever blocking the scan loop (was the cause of
#    blur at frame ~600 when the queue saturated).
#
# 5. ASYNC LOGGER — replaced logger.info() in hot path with a
#    lightweight non-blocking print + dedicated log thread.
#    logger.info() can block on file I/O under load.
# ============================================

import time
import threading
import queue as stdlib_queue

from motor_gcode import Motor
from camera import Camera
from logger import get_logger
from autofocus import Focus
import os

# ================= CONFIG =================
ROWS = 32
COLS = 32

X_STEP = -0.25
Y_STEP = -0.25

X_START = -12.0
Y_START = -17.0
Z_START = -1.90

SAVE_ROOT = "/mnt/ssd/LUT_1024/Scan_Output_Cervical_Sample/"

ANCHOR_IX = [0, 15.5, 31]
ANCHOR_IY = [0, 15.5, 31]

Z_ANCHOR = [[0.0]*3 for _ in range(3)]

# Settle time after Z move + M400. Safe at 80ms with locked exposure.
# If you see any residual blur, bump to 100ms — NOT back to 250ms.
SETTLE_MS = 0.080

# ================= ASYNC LOG QUEUE =================
# Logger.info() can block on file I/O. We push log strings to a thread
# so the scan loop never waits on disk.
_log_q = stdlib_queue.Queue()

def _log_worker(logger):
    while True:
        msg = _log_q.get()
        if msg is None:
            break
        logger.info(msg)

def async_log(msg):
    _log_q.put_nowait(msg)

# ================= TIMERS ==================
t_total_start = time.time()

# ================= INIT ==================
motor = Motor("/dev/ttyACM0", 115200)
motor.home_all()
time.sleep(0.4)

motor.send_gcode("M42 P1 S1.0")

motor.move_xyz_u(x=X_START, y=Y_START)
time.sleep(0.3)
motor.move_xyz_u(z=Z_START)

x_rel = X_START
y_rel = Y_START
z_rel = Z_START

camera = Camera()
camera.start()

focus  = Focus(camera, motor)
logger = get_logger("scan")

# Start async log worker
_log_thread = threading.Thread(target=_log_worker, args=(logger,), daemon=True)
_log_thread.start()


def get_next_pid_dir(root_path):
    max_pid = 0
    if os.path.exists(root_path):
        for name in os.listdir(root_path):
            if name.startswith("cervical_pid_"):
                try:
                    pid = int(name.split("_")[2])
                    max_pid = max(max_pid, pid)
                except:
                    continue
    next_pid = max_pid + 1
    return os.path.join(root_path, f"cervical_pid_{next_pid:02d}_{ROWS}_{COLS}")


# ==========================================================
# COARSE INITIAL Z SEARCH
# ==========================================================
def find_initial_z():
    logger.info("[Initial Z] Coarse search starting...")

    max_var = 0
    best_z  = motor.z

    step  = -0.01
    steps = 40

    for i in range(steps):
        motor.move_xyz_u(z=step)
        time.sleep(0.15)

        img  = camera.capture_lowres_for_autofocus()
        gray = focus.preprocess(img)
        var  = focus.variance(gray)

        logger.info(f"[Initial Z] Step {i+1:02d}/{steps} | Z={motor.z:.4f} | Var={var:.2f}")

        if var > max_var:
            max_var = var
            best_z  = motor.z

    motor.move_xyz_u(z=best_z - motor.z)
    time.sleep(0.1)
    return motor.z


# ==========================================================
# BUILD Z SURFACE
# ==========================================================
def build_z_anchor_table():
    global x_rel, y_rel

    print("\n[Z MAP] Building 3x3 surface...\n")
    prev_best = motor.z

    for r in range(3):
        col_range = range(3) if r % 2 == 0 else range(2, -1, -1)

        for c in col_range:
            target_x = X_START + ANCHOR_IX[c] * X_STEP
            target_y = Y_START + ANCHOR_IY[r] * Y_STEP

            dx = target_x - x_rel
            dy = target_y - y_rel

            motor.move_xyz_u(x=dx, y=dy)
            x_rel += dx
            y_rel += dy

            time.sleep(0.5)

            if r == 0 and c == 0:
                best_z  = find_initial_z()
                af_time = None
            else:
                best_z, _, af_time = focus.auto(prev_best)

            Z_ANCHOR[r][c] = round(best_z, 5)
            prev_best = best_z

            if af_time is not None:
                print(f"[AF] Anchor ({r},{c}) autofocus time: {af_time:.2f}s")

    print("\nFinal Z_ANCHOR:")
    for row in Z_ANCHOR:
        print(row)

    dx = X_START - x_rel
    dy = Y_START - y_rel
    motor.move_xyz_u(x=dx, y=dy)
    x_rel += dx
    y_rel += dy

    motor.move_xyz_u(z=Z_ANCHOR[0][0] - motor.z)


# ==========================================================
# BILINEAR INTERPOLATION
# ==========================================================
def get_z(ix, iy):
    if ix <= ANCHOR_IX[1]:
        cx = 0
        tx = (ix - ANCHOR_IX[0]) / (ANCHOR_IX[1] - ANCHOR_IX[0])
    else:
        cx = 1
        tx = (ix - ANCHOR_IX[1]) / (ANCHOR_IX[2] - ANCHOR_IX[1])

    if iy <= ANCHOR_IY[1]:
        cy = 0
        ty = (iy - ANCHOR_IY[0]) / (ANCHOR_IY[1] - ANCHOR_IY[0])
    else:
        cy = 1
        ty = (iy - ANCHOR_IY[1]) / (ANCHOR_IY[2] - ANCHOR_IY[1])

    z00 = Z_ANCHOR[cy][cx]
    z10 = Z_ANCHOR[cy][cx+1]
    z01 = Z_ANCHOR[cy+1][cx]
    z11 = Z_ANCHOR[cy+1][cx+1]

    return round(
        z00*(1-tx)*(1-ty) +
        z10*tx*(1-ty) +
        z01*(1-tx)*ty +
        z11*tx*ty,
        5
    )


# ================= BUILD SURFACE ==================
t_anchor_start = time.time()
build_z_anchor_table()
t_anchor_end     = time.time()
t_anchor_elapsed = t_anchor_end - t_anchor_start
print(f"\n[TIMER] 9-Point Anchor Table  →  {t_anchor_elapsed:.2f}s  ({t_anchor_elapsed/60:.2f} min)\n")

# =========================================================
# FIX: Lock AEC/AGC BEFORE scan starts
# This freezes exposure + gain so the ISP never adjusts
# between sensor rows — eliminating rolling shutter skew.
# =========================================================
camera.lock_exposure_for_scan()

t_scan_start = time.time()

# ================= SCAN ==================
try:
    save_dir = get_next_pid_dir(SAVE_ROOT)
    os.makedirs(save_dir, exist_ok=True)
    logger.info(f"[SCAN] Saving to: {save_dir}")

    for row in range(ROWS):

        if row % 2 == 0:
            col_indices = range(COLS)
            y_step = Y_STEP
        else:
            col_indices = range(COLS - 1, -1, -1)
            y_step = -Y_STEP

        pending_y_move = False  # tracks whether a non-blocking Y move is in-flight

        for step_i, col in enumerate(col_indices):

            # --------------------------------------------------
            # OPTIMISATION: flush the pipelined Y move from the
            # PREVIOUS iteration here, instead of waiting for it
            # with blocking call. By now (after Z calc + log) the
            # motor has had ~20-30ms head-start and is likely done.
            # --------------------------------------------------
            if pending_y_move:
                motor.flush_move()
                pending_y_move = False

            ix = row
            iy = col

            z_target = get_z(ix, iy)
            dz = z_target - motor.z

            # Z move: always blocking (M400) — we need the stage settled
            if dz != 0:
                motor.move_xyz_u(z=dz, wait=True)

            # Settle: 80ms is safe with locked exposure.
            # The ISP is no longer adjusting so there is no exposure
            # ramp-up; mechanical vibration from a 10-micron Z step
            # damps within ~30ms on a rigid stage.
            time.sleep(SETTLE_MS)

            # ---- CAPTURE ----
            serp_index = (row * COLS + col) if row % 2 == 0 else (row * COLS + (COLS - 1 - col))
            filename   = f"cervical_{serp_index:04d}.tiff"
            filepath   = os.path.join(save_dir, filename)

            img_path, q_depth = camera.capture_fullres_image_to(filepath)

            # Warn if writer queue is backing up (> 75% full → 192/256)
            if q_depth > 192:
                print(f"[WARN] Writer queue at {q_depth}/256 — frame {serp_index}")

            # Push log off the critical path
            async_log(
                f"[SCAN] row={row} col={col} Z={motor.z:.5f} "
                f"q={q_depth} saved={img_path}"
            )

            # --------------------------------------------------
            # OPTIMISATION: issue Y move NON-BLOCKING right after
            # capture. The motor starts moving immediately while
            # Python handles logging and the next iteration's Z calc.
            # flush_move() at the TOP of the next iteration ensures
            # it has completed before we capture again.
            # --------------------------------------------------
            if step_i < COLS - 1:
                motor.move_xyz_u(y=y_step, wait=False)
                y_rel += y_step
                pending_y_move = True

        # End of row: wait for any in-flight Y move before moving X
        if pending_y_move:
            motor.flush_move()

        if row < ROWS - 1:
            motor.move_xyz_u(x=X_STEP, wait=True)
            x_rel += X_STEP

finally:
    # Stop async log thread
    _log_q.put(None)
    _log_thread.join(timeout=5)

    t_scan_end     = time.time()
    t_scan_elapsed = t_scan_end - t_scan_start
    t_total_elapsed = t_scan_end - t_total_start

    print(f"\n[TIMER] LUT 1024 Scan          →  {t_scan_elapsed:.2f}s  ({t_scan_elapsed/60:.2f} min)")
    print(f"[TIMER] 9-Point Anchor Table   →  {t_anchor_elapsed:.2f}s  ({t_anchor_elapsed/60:.2f} min)")
    print(f"[TIMER] Total Script           →  {t_total_elapsed:.2f}s  ({t_total_elapsed/60:.2f} min)")

    camera.unlock_exposure()
    camera.stop()
    motor.release()
    logger.info("Scan finished")