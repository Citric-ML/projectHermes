from sensors.sonar import get_distance
from sensors.camera import capture_image
from Grid_building.layoccgrid import LayeredOccupancyGrid

#-----------------
#Helper functiones
#-----------------

import numpy as np

def compute_scale(depth_map, sonar_depth, pixel=(133, 133),
                  window_size=5,
                  min_scale=0.5, max_scale=2.5,
                  min_camera_depth=1e-6):
    """
    Computes a global depth scaling factor using sonar reference.

    Parameters:
        depth_map (np.ndarray): 2D array from MiDaS (relative depth).
        sonar_depth (float): Measured sonar depth in meters.
        pixel (tuple): (row, col) index aligned with sonar direction.
        min_scale (float): Minimum allowed scale factor.
        max_scale (float): Maximum allowed scale factor.
        min_camera_depth (float): Threshold to prevent division by zero.

    Returns:
        float or None: Valid scale factor, or None if invalid.
    """

    if sonar_depth is None or sonar_depth <= 0:
        return None

    h, w = depth_map.shape
    r, c = pixel

    if not (0 <= r < h and 0 <= c < w):
        return None

    # ---- Extract window around center pixel ----
    half = window_size // 2

    r0 = max(0, r - half)
    r1 = min(h, r + half + 1)
    c0 = max(0, c - half)
    c1 = min(w, c + half + 1)

    window = depth_map[r0:r1, c0:c1]

    # Filter invalid values
    valid = window[np.isfinite(window)]
    valid = valid[valid > min_camera_depth]

    if len(valid) == 0:
        return None

    camera_depth = np.median(valid)

    scale = sonar_depth / camera_depth

    if not (min_scale <= scale <= max_scale):
        return None

    return scale
#--------------------------------
#ML related stubs (work on later) URGENTTTTTTTTT!!!!1!!!!!11!
#--------------------------------
def estimate_depth(image):
    # Temporary placeholder depth map
    return np.ones((256, 256), dtype=np.float32)

def project_to_grid(depth_map, drone_pose, camera_params=None):
    """
    Converts a SCALED(must go through compute_scale() first) depth map into a hypothesis LayeredOccupancyGrid.

    Parameters:
        depth_map (np.ndarray): 2D depth array (meters) from model.
        drone_pose (dict): {x, y, z, yaw, pitch, roll}
        camera_params (dict): camera configuration

    Returns:
        LayeredOccupancyGrid: hypothesis grid

    [Note to self: later on only sample every N pixels, use vectorization, and downscale depth map for performance if needed]
    [Another note to self: grid.update() isn't using x/y/z_world, because right now LayeredOccupancyGrid() internally does raycasting based on direction/depth. Later this can be refactored to accept direct world hit coordinates if need be]
    """
    if depth_map is None:
        return None

    if camera_params is None:
        camera_params = {
            "width": depth_map.shape[1],
            "height": depth_map.shape[0],
            "fov_horizontal": 60.0,
            "fov_vertical": 45.0
        }

    grid = LayeredOccupancyGrid()

    height, width = depth_map.shape
    cx = width / 2
    cy = height / 2

    h_fov = np.radians(camera_params["fov_horizontal"])
    v_fov = np.radians(camera_params["fov_vertical"])

    yaw = np.radians(drone_pose["yaw"])

    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    for row in range(height):
        for col in range(width):

            depth = depth_map[row, col]

            if not np.isfinite(depth) or depth <= 0:
                continue

            # ---- Proper angle mapping ----
            x_angle = ((col - cx) / cx) * (h_fov / 2)
            y_angle = ((row - cy) / cy) * (v_fov / 2)

            # ---- Direction vector (camera frame) ----
            dx_cam = np.cos(y_angle) * np.sin(x_angle)
            dy_cam = np.sin(y_angle)
            dz_cam = np.cos(y_angle) * np.cos(x_angle)

            # ---- Scale by depth ----
            x_cam = dx_cam * depth
            y_cam = dy_cam * depth
            z_cam = dz_cam * depth

            # ---- Rotate into world frame (yaw only) ----
            x_world = drone_pose["x"] + (x_cam * cos_yaw - z_cam * sin_yaw)
            z_world = drone_pose["z"] + (x_cam * sin_yaw + z_cam * cos_yaw)
            y_world = drone_pose["y"] + y_cam  # assuming flat roll

            # ---- Update grid using world-space hit ----
            grid.update(
                drone_pos=(drone_pose["x"], drone_pose["z"], drone_pose["y"]),
                direction_deg=(drone_pose["yaw"], drone_pose["pitch"]),
                depth=depth
            )

    return grid

import time
#this is meant to be one of three concurent threads running on the Pi
def perception_loop(grid, get_drone_pose, get_direction):
    """
    Continuously updates the LayeredOccupancyGrid.

    grid: LayeredOccupancyGrid instance
    get_drone_pose(): returns (x, y, z)
    get_direction(): returns (yaw_deg, pitch_deg)
    """

    print("Perception loop started.")

    while True:
        loop_start = time.time()

        # 1. Capture frame
        frame = capture_image()
        if frame is None:
            continue

        # 2. Infer depth via MiDaS
        depth_map = estimate_depth(frame)
        if depth_map is None:
            continue

        # 3. Read sonar reference depth
        sonar_depth = get_distance()
        if sonar_depth is None:
            continue

        # 4. Compute scale
        scale = compute_scale(depth_map, sonar_depth)
        depth_map_scaled = depth_map * scale

        # 5. Extract center depth (example MVP)
        center_depth = depth_map_scaled[133, 133]

        # 6. Get drone pose + direction
        drone_pos = get_drone_pose()
        direction = get_direction()

        # 7. Update layered occupancy grid (raycast hit)
        grid.update(drone_pos, direction, center_depth)

        # Optional debug
        # grid.debug_print()

        loop_time = time.time() - loop_start
        print(f"Perception loop time: {loop_time:.3f}s")

        time.sleep(0.05)
