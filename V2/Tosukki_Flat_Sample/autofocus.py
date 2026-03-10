# from logger import get_logger
# log = get_logger(__name__)

import cv2
import time
import numpy as np
from PIL import Image

from motor_gcode import Motor
from camera import Camera


class Focus:
    def __init__(self, camera: Camera, motor: Motor):
        self.camera = camera
        self.motor = motor

    def preprocess(self, image):
        if isinstance(image, Image.Image):
            image = np.array(image)

        if image.ndim == 2:
            return image

        if image.shape[-1] == 4:
            image = image[:, :, :3]

        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def variance(self, gray):
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        return np.mean(gx * gx + gy * gy)

    # -------------------------------------------------
    # AUTOFOCUS WITH HASHMAP STORAGE
    # -------------------------------------------------
    ## version2 
    
    def auto(self, start_z, capture_fullres=False):
        """
        Systematic bidirectional autofocus scan.
    
        Process:
        1. Scan downward for N iterations (-step each time)
        2. Return to start position
        3. Scan upward for N iterations (+step each time)
        4. Select best focused image from all scans
        5. Move to best Z position
    
        Args:
            start_z: Starting Z position
            capture_fullres: If True, captures and stores full-res images
        
        Returns:
            tuple: (best_z, best_image, total_af_time)
        """
        step = 0.01  # 5 micron
        n_down = 10    # Number of downward steps
        n_up = 10      # Number of upward steps
    
        current_z = start_z
        best_z = start_z
        best_var = 0
    
        # HASHMAP: Store images with Z position as key
        focus_map = {}
    
        af_start = time.perf_counter()
        # log.info(f"[AF] Starting autofocus at Z={start_z:.4f}")
    
        # ====================================================
        # PHASE 1: DOWNWARD SCAN
        # ====================================================
        # log.info(f"[AF] Phase 1: Scanning DOWN {n_down} steps")
        for i in range(n_down):
            iter_start = time.perf_counter()
        
            # Move DOWN first
            dz = -step
            time.sleep(0.15)
            self.motor.move_xyz_u(z=dz)
            time.sleep(0.15)
            current_z += dz
        
            # Capture images
            t0 = time.perf_counter()
            img_lowres = self.camera.capture_lowres_for_autofocus()
            if capture_fullres:
                img_fullres = self.camera.capture_fullres_for_hashmap()
            else:
                img_fullres = img_lowres
            t_capture = time.perf_counter() - t0
        
            # Calculate variance
            gray = self.preprocess(img_lowres)
            var = self.variance(gray)
            
            # Store in hashmap
            focus_map[current_z] = {
                'image': img_fullres,
                'variance': var,
                'iteration': i,
                'phase': 'down'
            }
        
            # Track best
            if var > best_var:
                best_var = var
                best_z = current_z

            iter_time = time.perf_counter() - iter_start
            # log.info(
            #     f"[AF] DOWN {i+1}/{n_down} | Z={current_z:.4f} | Var={var:.2f} | "
            #     f"Capture={t_capture*1000:.1f}ms | Iter={iter_time*1000:.1f}ms"
            #         )
            time.sleep(0.15)

        # ====================================================
        # PHASE 2: RETURN TO START
        # ====================================================
        # log.info(f"[AF] Phase 2: Returning to start position Z={start_z:.4f}")
        return_move = start_z - current_z
        time.sleep(0.15)
        self.motor.move_xyz_u(z=return_move)
        time.sleep(0.15)
        current_z = start_z
        # log.info(f"[AF] Returned to Z={current_z:.4f}")
    
        # ====================================================
        # PHASE 3: UPWARD SCAN
        # ====================================================
        # log.info(f"[AF] Phase 3: Scanning UP {n_up} steps")
        for i in range(n_up):
            iter_start = time.perf_counter()
        
            # Move UP first
            dz = +step
            time.sleep(0.15)
            self.motor.move_xyz_u(z=dz)
            time.sleep(0.15)
            current_z += dz
        
            # Capture images
            t0 = time.perf_counter()
            img_lowres = self.camera.capture_lowres_for_autofocus()
            if capture_fullres:
                img_fullres = self.camera.capture_fullres_for_hashmap()
            else:
                img_fullres = img_lowres
            t_capture = time.perf_counter() - t0
        
            # Calculate variance
            gray = self.preprocess(img_lowres)
            var = self.variance(gray)
        
            # Store in hashmap
            focus_map[current_z] = {
                'image': img_fullres,
                'variance': var,
                'iteration': i + n_down,
                'phase': 'up'
            }
        
            # Track best
            if var > best_var:
                best_var = var
                best_z = current_z
        
            iter_time = time.perf_counter() - iter_start
            # log.info(
            #     f"[AF] UP {i+1}/{n_up} | Z={current_z:.4f} | Var={var:.2f} | "
            #     f"Capture={t_capture*1000:.1f}ms | Iter={iter_time*1000:.1f}ms"
            # )
            time.sleep(0.15)
    
        # ====================================================
        # PHASE 4: MOVE TO BEST FOCUS
        # ====================================================
        # log.info(f"[AF] Phase 4: Moving to best focus position")
        correction_move = best_z - current_z
        time.sleep(0.15)
        self.motor.move_xyz_u(z=correction_move)
        time.sleep(0.15)
    
        total_af_time = time.perf_counter() - af_start
    
        # Retrieve best focused image
        best_image = focus_map[best_z]['image']
    
        # log.info(f"[AF] ========================================")
        # log.info(f"[AF] Best focus Z = {best_z:.4f} | Var = {best_var:.2f}")
        # log.info(f"[AF] Final correction: {correction_move:+.4f}mm")
        # log.info(f"[AF] Images cached: {len(focus_map)}")
        # log.info(f"[AF] Total autofocus time: {total_af_time:.2f}s")
        # log.info(f"[AF] ========================================")
    
        return best_z, best_image, total_af_time



    ## version1 

    """
    def auto(self, start_z, capture_fullres=True):
        
        #Iterative autofocus with hashmap storage.
        
        #Your original robust logic:
        #- Adaptive step direction based on variance trend
        #- Backlash compensation
        #- Convergence detection
        
        #NEW: Hashmap storage
        #- Stores full-res images at each Z position
        #- Returns best focused image after convergence
        
        #Args:
        #    start_z: Starting Z position
        #    capture_fullres: If True, captures and stores full-res images
            
        #Returns:
        #    tuple: (best_z, best_image, total_af_time)
        
        step = 0.005 ## 5 micron
        backlash = 0.00  ## 5 micron
        threshold = 10
        max_iter = 20     ## iteration T1 10 steps
        initial_steps = 2

        current_z = start_z
        best_z = start_z
        best_var = 0

        variances = []
        prev_direction = None
        
        #  HASHMAP: Store images with Z position as key
        focus_map = {}

        af_start = time.perf_counter()
        log.info(f"[AF] Starting autofocus at Z={start_z:.4f}")

        for i in range(max_iter):
            iter_start = time.perf_counter()

            # -------- Image capture timing --------
            t0 = time.perf_counter()
            
            # Capture low-res for variance calculation
            img_lowres = self.camera.capture_lowres_for_autofocus()
            
            # Optionally capture full-res for hashmap storage
            if capture_fullres:
                img_fullres = self.camera.capture_fullres_for_hashmap()
            else:
                img_fullres = img_lowres
                
            t_capture = time.perf_counter() - t0

            # Calculate variance from low-res image
            gray = self.preprocess(img_lowres)
            var = self.variance(gray)
            variances.append(var)

            # HASHMAP: Store image with its variance at current Z
            focus_map[current_z] = {
                'image': img_fullres,
                'variance': var,
                'iteration': i
            }

            # Track best focus
            if var > best_var:
                best_var = var
                best_z = current_z

            # -------- Decide Z movement --------
            if i < initial_steps:
                dz = -step
            elif i == initial_steps:
                dz = 2 * step
            elif i < 2 * initial_steps:
                dz = step
            else:
                direction = 1 if variances[-1] > variances[-2] else -1
                
                # BUG FIX: Track backlash compensation in current_z
                if prev_direction and direction != prev_direction:
                    backlash_move = backlash * direction
                    time.sleep(0.15)
                    self.motor.move_xyz_u(z=backlash_move)
                    time.sleep(0.15)
                    current_z += backlash_move  # FIXED: Update current_z!
                    log.info(f"[AF] Backlash compensation: {backlash_move:+.4f}mm, Z now={current_z:.4f}")
                
                dz = step * direction
                prev_direction = direction

            # -------- Z move timing --------
            t1 = time.perf_counter()
            time.sleep(0.15)
            self.motor.move_xyz_u(z=dz)
            time.sleep(0.15)
            t_move = time.perf_counter() - t1

            current_z += dz

            iter_time = time.perf_counter() - iter_start

            log.info(
                f"[AF] Iter {i+1} | ΔZ={dz:+.4f} | Z={current_z:.4f} | "
                f"Var={var:.2f} | "
                f"Capture={t_capture*1000:.1f}ms | "
                f"Move={t_move*1000:.1f}ms | "
                f"Iter={iter_time*1000:.1f}ms"
            )

            time.sleep(0.15)

            # Convergence detection
            if i > 3:
                if (
                    abs(variances[-1] - variances[-2]) < threshold
                    and abs(variances[-3] - variances[-4]) < threshold
                ):
                    log.info("[AF] Variance stabilized → stopping")
                    break

        # -------- Final correction timing --------
        t_corr = time.perf_counter()
        correction_move = best_z - current_z
        time.sleep(0.15)
        self.motor.move_xyz_u(z=correction_move)
        time.sleep(0.15)
        corr_time = time.perf_counter() - t_corr

        total_af_time = time.perf_counter() - af_start

        #  HASHMAP: Retrieve best focused image
        best_image = focus_map[best_z]['image']

        log.info(f"[AF] Best focus Z = {best_z:.4f} | Var = {best_var:.2f}")
        log.info(f"[AF] Final correction: {correction_move:+.4f}mm")
        log.info(f"[AF] Images cached: {len(focus_map)}")
        log.info(f"[AF] Final correction time: {corr_time*1000:.1f}ms")
        log.info(f"[AF] Total autofocus time: {total_af_time:.2f}s")

        return best_z, best_image, total_af_time
        """
