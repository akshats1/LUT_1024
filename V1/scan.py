# ============================================
# SERPENTINE SCAN WITH BILINEAR Z
# 32x32 = 1024 FOV, 0.3mm steps
# ============================================

import time
from motor_gcode import Motor
from camera import Camera
from logger import get_logger

# ================= CONFIG =================
ROWS = 32   # X direction
COLS = 32   # Y direction

X_STEP = -0.3
Y_STEP = -0.3

X_START = -7.0
Y_START = -13.5
#Z_START = -1.71   # Z at anchor A (first point)
Z_START = -1.74   # Z at anchor A (first point)

SAVE_DIR = "/mnt/ssd/LUT_1024/Scan_Output/"

# ================= Z ANCHOR TABLE =================
# 9 measured focus points on sample surface
# Z_ANCHOR[row_index][col_index]
# row0 = top    (y=-13.5 ),  row1 = middle (y=-17.85), row2 = bottom (y=-22.2)
# col0 = left   (x=-7.0  ),  col1 = middle (x=-10.85), col2 = right  (x=-14.7)

"""
Z anchor value is Hardcoded now
need to fetch dynamically
"""

# Z_ANCHOR = [
#     [-1.71, -1.74, -1.79],   # row0: A, B, C
#     [-1.65, -1.67, -1.70],   # row1: F, E, D
#     [-1.65, -1.68, -1.69],   # row2: G, H, I
# ]
Z_ANCHOR = [
    [-1.74, -1.78, -1.82],   # row0: A, B, C
    [-1.73, -1.7, -1.68],   # row1: F, E, D
    [-1.68, -1.7, -1.71],   # row2: G, H, I
]

# Where the 3 anchor columns/rows sit in 0..31 grid index space
ANCHOR_IX = [0, 15.5, 31]   # col0, col1, col2
ANCHOR_IY = [0, 15.5, 31]   # row0, row1, row2

# ================= Z INTERPOLATION =================
def get_z(ix, iy):
    """Bilinear Z interpolation at grid position ix (col), iy (row)"""
    # find X cell and fraction
    if ix <= ANCHOR_IX[1]:
        cx, tx = 0, (ix - ANCHOR_IX[0]) / (ANCHOR_IX[1] - ANCHOR_IX[0])
    else:
        cx, tx = 1, (ix - ANCHOR_IX[1]) / (ANCHOR_IX[2] - ANCHOR_IX[1])

    # find Y cell and fraction
    if iy <= ANCHOR_IY[1]:
        cy, ty = 0, (iy - ANCHOR_IY[0]) / (ANCHOR_IY[1] - ANCHOR_IY[0])
    else:
        cy, ty = 1, (iy - ANCHOR_IY[1]) / (ANCHOR_IY[2] - ANCHOR_IY[1])

    z00 = Z_ANCHOR[cy    ][cx    ]
    z10 = Z_ANCHOR[cy    ][cx + 1]
    z01 = Z_ANCHOR[cy + 1][cx    ]
    z11 = Z_ANCHOR[cy + 1][cx + 1]

    return round(z00*(1-tx)*(1-ty) + z10*tx*(1-ty) +
                 z01*(1-tx)*ty     + z11*tx*ty, 5)

# ================= INIT ==================
motor = Motor("/dev/ttyACM0", 115200)
motor.home_all()
time.sleep(1)

motor.send_gcode("M42 P1 S1.0")   # LED ON

# -------------------------------------------------------
# FIX: After home_all() motor is at machine (0,0,0).
# Use G90 absolute mode to move to actual sample start.
# Then restore G91 relative mode for the scan loop.
# -------------------------------------------------------
motor.send_gcode("G90", wait=True)
motor.send_gcode(f"G1 X{X_START} Y{Y_START} F300", wait=True)       # XY first
motor.send_gcode(f"G1 Z{Z_START} F100", wait=True)                  # then Z
motor.send_gcode("G91", wait=True)                                  # back to relative
motor.z = Z_START                                                   # sync Z tracker
time.sleep(1)
print(f"[SCAN] At start: X={X_START}  Y={Y_START}  Z={Z_START}")

camera = Camera()
camera.start()
time.sleep(1)

logger = get_logger("scan")

# ================= STATE =================
x_rel = 0.0
y_rel = 0.0
z_rel = Z_START

# ================= SCAN ==================
try:
    for row in range(ROWS):

        # -------- serpentine Y direction ----------
        if row % 2 == 0:
            col_indices = range(COLS)
            y_step = Y_STEP
        else:
            col_indices = range(COLS - 1, -1, -1)
            y_step = -Y_STEP

        for step_in_row, col in enumerate(col_indices):

            # grid indices for Z lookup
            ix = row    # row = X axis index (0..31)
            iy = col    # col = Y axis index (0..31)

            # get target Z from bilinear interpolation
            z_target = get_z(ix, iy)
            dz = round(z_target - z_rel, 5)

            # -------- move Z if needed ----------
            if dz != 0:
                motor.move_xyz_u(z=dz)
                z_rel = z_target

            time.sleep(0.5)

            # -------- capture image ----------
            img_path = camera.capture_fullres_image(SAVE_DIR)

            logger.info(
                f"[SCAN] row={row} col={col} "
                f"X={X_START + x_rel:.3f} "
                f"Y={Y_START + y_rel:.3f} "
                f"Z={z_rel:.5f} "
                f"saved={img_path}"
            )

            # -------- move Y except last step ----------
            if step_in_row < COLS - 1:
                motor.move_xyz_u(y=y_step)
                y_rel += y_step

        # -------- move X to next row ----------
        if row < ROWS - 1:
            motor.move_xyz_u(x=X_STEP)
            x_rel += X_STEP

# ================= CLEANUP =================
### releasing all resources
finally:
    camera.stop()
    motor.release()
    logger.info("Scan finished")
