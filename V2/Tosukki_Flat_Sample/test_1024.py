import numpy as np
import pandas as pd

ROWS = 32
COLS = 32

ANCHOR_IX = [0, 15.5, 31]
ANCHOR_IY = [0, 15.5, 31]

Z_ANCHOR = [
    [-1.79, -1.82, -1.81],
    [-1.71, -1.72, -1.71],
    [-1.72, -1.73, -1.68]
]

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


z_map = np.zeros((ROWS, COLS))

for i in range(ROWS):
    for j in range(COLS):
        z_map[i,j] = get_z(i, j)

df = pd.DataFrame(z_map)
df.to_excel("z_surface_1024.xlsx", index=False)

print("1024 Z values generated")