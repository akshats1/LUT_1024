# LUT Scan Logic 
## Layout

```bash
(0,0) -------- (0,15.5) -------- (0,31)
  |               |               |
  |    Cell 1     |    Cell 2     |
  |               |               |
(15.5,0) ----- (15.5,15.5) ---- (15.5,31)
  |               |               |
  |    Cell 3     |    Cell 4     |
  |               |               |
(31,0) -------- (31,15.5) ------- (31,31)
```

## Final Structure
```bash

        32 columns
 ┌───────────────┬───────────────┐
 │   Cell 1      │   Cell 2      │
 │   16×16=256   │   16×16=256   │
 │               │               │
 ├───────────────┼───────────────┤
 │   Cell 3      │   Cell 4      │
 │   16×16=256   │   16×16=256   │
 │               │               │
 └───────────────┴───────────────┘
         32 rows
```

## Bilinear Calculation
```bash
(z00) ----- (z10) ----- (z20)
  |           |           |
  |   cell 1  | cell 2    |
  |           |           |
(z01) ----- (z11) ----- (z21)
  |           |           |
  |  cell 3   |   cell 4  |
  |           |           |
(z02) ----- (z12) ----- (z22)
```

## Determine Current Cell

```bash
for row in range(32):
    for col in range(32):
if row <= 15.5 → top cells
else → bottom cells

if col <= 15.5 → left cells
else → right cells

```
```bash
** Selects the four surrounding anchor points:
z00, z10
z01, z11
```
## Compute Interpolation Fractions
```bash
tx = horizontal distance ratio
ty = vertical distance ratio

```
```bash
tx = (col - anchor_left) / (anchor_right - anchor_left)

ty = (row - anchor_top) / (anchor_bottom - anchor_top)
```
```bash
tx = 0 → left anchor
tx = 1 → right anchor

ty = 0 → top anchor
ty = 1 → bottom anchor
```

## Bilinear Z Calculation 

```bash
Z = z00*(1-tx)*(1-ty)
  + z10*(tx)*(1-ty)
  + z01*(1-tx)*(ty)
  + z11*(tx)*(ty)
```
```bash
*** Z Coordinate Description ***
z00 = top-left anchor
z10 = top-right anchor
z01 = bottom-left anchor
z11 = bottom-right anchor
```

## Stage Movement

```bash
1.Move stage in X–Y direction

2.Calculate target Z using bilinear interpolation

3.Compute relative Z move
```
### Compute relative Z move
```bash
dz = target_z - current_z
```
### Move Z axis
```bash
motor.move_xyz_u(z = dz)
```

### Capture image
```bash
camera.capture_fullres_image()
```