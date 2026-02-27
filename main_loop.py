from sensors.sonar import get_distance
from sensors.camera import capture_image
from Grid_building.layoccgrid import LayeredOccupancyGrid
from Grid_building.layoccgrid import StaticOccupancyGrid
from midas import DepthEstimator
from arduino_communication import build_command_list
from Grid_building.converged import buildpathcoords
from arduino_communication import DroneController
from Grid_building.layoccgrid import promote_static_cells
from sensors.accelerometer import MPU6050
from sensors.accelerometer import IMUTracker

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
#ML related stubs
#--------------------------------

def project_to_grid(depth_map, drone_pose, camera_params=None, sample_step=4):
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

    for row in range(0, height, sample_step):
        for col in range(0, width, sample_step):

            depth = depth_map[row, col]

            if not np.isfinite(depth) or depth <= 0:
                continue

            # Pixel -> angle
            x_angle = ((col - cx) / cx) * (h_fov / 2)
            y_angle = ((row - cy) / cy) * (v_fov / 2)

            # Camera frame direction
            dx_cam = np.cos(y_angle) * np.sin(x_angle)
            dy_cam = np.sin(y_angle)
            dz_cam = np.cos(y_angle) * np.cos(x_angle)

            # Scale by depth
            x_cam = dx_cam * depth
            y_cam = dy_cam * depth
            z_cam = dz_cam * depth

            # Rotate into world frame (yaw only)
            x_world = drone_pose["x"] + (x_cam * cos_yaw - z_cam * sin_yaw)
            y_world = drone_pose["y"] + y_cam
            z_world = drone_pose["z"] + (x_cam * sin_yaw + z_cam * cos_yaw)

            #Use direct world coordinate update
            grid.update_world_hit(x_world, y_world, z_world)

    return grid


import time
#this is meant to be one of three concurent threads running on the Pi
def main_loop():

    print("System started.")

    # Persistent systems
    depth_estimator = DepthEstimator()
    imu = MPU6050()
    tracker = IMUTracker()
    tracker.calibrate(imu) #sets biases for accelerometer
    dynamic_grid = LayeredOccupancyGrid()
    static_grid = StaticOccupancyGrid()
    controller = DroneController()

    while True:

        loop_start = time.time()

        # -------------------------------------------------
        # 1. POSITION UPDATE

        ax, ay, az = imu.get_accel()
        gx, gy, gz = imu.get_gyro()

        tracker.update(ax, ay, az, gx, gy, gz)

        current_pose = tracker.get_pose()
        drone_pose = current_pose
        # -------------------------------------------------
        # 2. PERCEPTION

        frame = capture_image()
        if frame is None:
            controller.listen()
            continue

        depth_map = depth_estimator.predict(frame)
        if depth_map is None:
            controller.listen()
            continue

        sonar_depth = get_distance()
        if sonar_depth is None:
            controller.listen()
            continue

        scale = compute_scale(depth_map, sonar_depth)
        if scale is None:
            controller.listen()
            continue

        depth_map_scaled = depth_map * scale

        hypothesis_grid = project_to_grid(
            depth_map_scaled,
            drone_pose=drone_pose,
            camera_params=None,
            sample_step=4
        )

        # -------------------------------------------------
        # 2. MAPPING

        dynamic_grid.merge(hypothesis_grid)
        dynamic_grid.decay()

        promote_static_cells(
            dynamic_grid,
            static_grid
        )

        # -------------------------------------------------
        # 3. PLANNING

        #this function also builds the merged projection
        path = buildpathcoords()

        if path:
            commands = build_command_list(path)
            controller.update_commands(commands)

        # -------------------------------------------------
        # 4. COMMUNICATION

        controller.listen()

        # -------------------------------------------------
        # 5. LOOP TIMING

        elapsed = time.time() - loop_start
        sleep_time = max(0.05 - elapsed, 0)
        time.sleep(sleep_time)
'''
You need to do these things:
1. Fix Astar so that it has a set goal and startpoint
'''
