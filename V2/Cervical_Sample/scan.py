# ============================================
# SERPENTINE SCAN - PURE RELATIVE VERSION
# Dynamic 3x3 Z surface
# No G90
# No preview
# ============================================

import time
from motor_gcode import Motor
from camera import Camera
from logger import get_logger
from autofocus import Focus

# ================= CONFIG =================
ROWS = 32
COLS = 32

X_STEP = -0.3
Y_STEP = -0.3

X_START = -12.0
Y_START = -16.0
Z_START = -1.60

SAVE_DIR = "/mnt/ssd/LUT_1024/Scan_Output_Cervical/"

ANCHOR_IX = [0, 15.5, 31]
ANCHOR_IY = [0, 15.5, 31]

Z_ANCHOR = [[0.0]*3 for _ in range(3)]

# ================= INIT ==================
motor = Motor("/dev/ttyACM0", 115200)
motor.home_all()
time.sleep(1)

motor.send_gcode("M42 P1 S1.0")

# Move to start RELATIVELY
motor.move_xyz_u(x=X_START, y=Y_START, z=Z_START)
motor.z = Z_START

# Track relative position in software
x_rel = X_START
y_rel = Y_START
z_rel = Z_START

camera = Camera()
camera.start()

focus = Focus(camera, motor)
logger = get_logger("scan")

# ==========================================================
# COARSE INITIAL Z SEARCH
# ==========================================================
def find_initial_z():

    logger.info("[Initial Z] Coarse search starting...")

    max_var = 0
    best_z = motor.z

    step = -0.05
    steps = 10

    for i in range(steps):

        img = camera.capture_lowres_for_autofocus()
        gray = focus.preprocess(img)
        var = focus.variance(gray)

        logger.info(f"[Initial Z] Step {i+1} | Z={motor.z:.4f} | Var={var:.2f}")

        if var > max_var:
            max_var = var
            best_z = motor.z

        motor.move_xyz_u(z=step)
        time.sleep(0.2)

    # Move back to best Z
    correction = best_z - motor.z
    motor.move_xyz_u(z=correction)
    motor.z = best_z

    return best_z


# ==========================================================
# BUILD Z SURFACE - PURE RELATIVE
# ==========================================================
def build_z_anchor_table():

    global x_rel, y_rel

    print("\n[Z MAP] Building 3x3 surface...\n")

    prev_best = motor.z

    for r in range(3):
        # Serpentine: reverse column order on odd rows
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
                best_z = find_initial_z()
            else:
                best_z, _, _ = focus.auto(prev_best)

            Z_ANCHOR[r][c] = round(best_z, 5)
            prev_best = best_z

            print(f"[Z MAP] Anchor ({r},{c}) Z={best_z:.5f}")

    print("\nFinal Z_ANCHOR:")
    for row in Z_ANCHOR:
        print(row)

    # Return to scan start
    dx = X_START - x_rel
    dy = Y_START - y_rel

    motor.move_xyz_u(x=dx, y=dy)
    x_rel += dx
    y_rel += dy

    # Move to first anchor Z
    dz = Z_ANCHOR[0][0] - motor.z
    motor.move_xyz_u(z=dz)
    motor.z = Z_ANCHOR[0][0]


# The traversal pattern now looks like this:
# ```
# Row 0 (even):  (0,0) → (0,1) → (0,2)
# Row 1 (odd):   (1,2) → (1,1) → (1,0)
# Row 2 (even):  (2,0) → (2,1) → (2,2)


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
## 9 Points Table {3X3}
build_z_anchor_table()




# ================= SCAN ==================
try:
    for row in range(ROWS):

        if row % 2 == 0:
            col_indices = range(COLS)
            y_step = Y_STEP
        else:
            col_indices = range(COLS - 1, -1, -1)
            y_step = -Y_STEP

        for step_i, col in enumerate(col_indices):

            ix = row
            iy = col

            z_target = get_z(ix, iy)
            dz = z_target - motor.z

            if dz != 0:
                motor.move_xyz_u(z=dz)

            time.sleep(0.3)

            img_path = camera.capture_fullres_image(SAVE_DIR)

            logger.info(
                f"[SCAN] row={row} col={col} "
                f"Z={motor.z:.5f} saved={img_path}"
            )

            if step_i < COLS - 1:
                motor.move_xyz_u(y=y_step)
                y_rel += y_step

        if row < ROWS - 1:
            motor.move_xyz_u(x=X_STEP)
            x_rel += X_STEP


finally:
    camera.stop()
    motor.release()
    logger.info("Scan finished")