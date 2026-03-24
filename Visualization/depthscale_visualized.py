"""
Depth Map + Sonar ASCII Visualizer
===================================
Renders two kinds of ASCII grids:

  1. Depth grid  - a top-down side-profile view of the MiDaS relative depth map.
                   The drone position is marked at the bottom-centre with "^".
                   Each column of the grid represents one horizontal slice of the
                   camera frame. Cell brightness encodes depth (0=close, 9=far).

  2. Sonar overlay - same grid but with a column of "|" characters drawn
                     straight up from the drone marker to the row that matches
                     the sonar-reported distance. The sonar beam endpoint is
                     marked with "S". The rest of the depth map is scaled by
                     compute_scale() before rendering so everything is in metres.

Only standard ASCII characters are used (no Unicode).
"""

import numpy as np

# --------------------------------------------
# Scale helper (matches compute_scale in main_loop.py exactly)
# --------------------------------------------

def compute_scale(depth_map, sonar_depth, pixel=None,
                  window_size=5,
                  min_scale=0.5, max_scale=2.5,
                  min_camera_depth=1e-6):
    """
    Computes a global depth scaling factor using a sonar reference point.

    Parameters:
        depth_map       : np.ndarray (H, W) - raw MiDaS relative depth output.
        sonar_depth     : float - measured forward distance in metres.
        pixel           : (row, col) of the pixel aligned with the sonar beam.
                          Defaults to the image centre if None.
        window_size     : int - size of the median window around the pixel.
        min_scale       : float - lowest accepted scale factor.
        max_scale       : float - highest accepted scale factor.
        min_camera_depth: float - guard against division by near-zero values.

    Returns:
        float or None - valid scale factor, or None if inputs are invalid or
                        the computed scale falls outside [min_scale, max_scale].
    """
    if sonar_depth is None or sonar_depth <= 0:
        return None

    h, w = depth_map.shape

    if pixel is None:
        pixel = (h // 2, w // 2)

    r, c = pixel

    if not (0 <= r < h and 0 <= c < w):
        return None

    half = window_size // 2
    r0 = max(0, r - half)
    r1 = min(h, r + half + 1)
    c0 = max(0, c - half)
    c1 = min(w, c + half + 1)

    window = depth_map[r0:r1, c0:c1]
    valid  = window[np.isfinite(window)]
    valid  = valid[valid > min_camera_depth]

    if len(valid) == 0:
        return None

    camera_depth = np.median(valid)
    scale        = sonar_depth / camera_depth

    if not (min_scale <= scale <= max_scale):
        return None

    return scale


# --------------------------------------------
# Depth map -> column profile
# --------------------------------------------
# The visualizer collapses the 2D depth map into a 1-D horizontal profile
# (one value per camera column) and then renders that profile as a 2-D ASCII
# grid where the Y axis is distance and the X axis is the camera's horizontal
# field of view.
#
# Mapping:
#   grid cols  = camera columns (after downsampling to GRID_COLS)
#   grid rows  = distance buckets from 0 (bottom = drone) to MAX_DEPTH (top)
#
# A cell is "filled" if the median depth for that camera column falls in or
# beyond that distance bucket.

GRID_COLS    = 40    # horizontal resolution of the ASCII grid
GRID_ROWS    = 20    # vertical resolution (distance axis)
MAX_DEPTH_M  = 8.0   # metres that map to the top row of the grid
DRONE_MARKER = '^'
SONAR_BEAM   = '|'
SONAR_TIP    = 'S'
EMPTY        = '.'
FILLED_CHARS = ' 123456789'   # index 0 = near (dark), 9 = far (bright)


def _depth_to_char(normalised_depth):
    """
    Map a 0-1 normalised depth to a display character.
    Near obstacles are shown as low digits, far ones as high digits.
    """
    idx = int(normalised_depth * (len(FILLED_CHARS) - 1))
    idx = max(0, min(idx, len(FILLED_CHARS) - 1))
    return FILLED_CHARS[idx]


def _build_column_profile(depth_map, grid_cols=GRID_COLS):
    """
    Collapse a (H, W) depth map into a 1-D array of length grid_cols.
    Each entry is the median depth (in whatever units depth_map uses) of all
    finite, positive pixels in that horizontal slice of the image.
    Returns a float array; entries are np.nan where no valid pixels exist.
    """
    h, w = depth_map.shape
    profile = np.full(grid_cols, np.nan)

    col_edges = np.linspace(0, w, grid_cols + 1).astype(int)

    for i in range(grid_cols):
        c0, c1 = col_edges[i], col_edges[i + 1]
        if c0 >= c1:
            continue
        slice_ = depth_map[:, c0:c1].ravel()
        valid  = slice_[np.isfinite(slice_) & (slice_ > 0)]
        if len(valid) > 0:
            profile[i] = np.median(valid)

    return profile


def _profile_to_grid(profile, max_depth, grid_rows=GRID_ROWS):
    """
    Convert a 1-D depth profile into a 2-D character grid.

    grid[row][col]:
      row 0   = top    = furthest distance
      row -1  = bottom = drone position (distance = 0)

    A column is "filled" up to the row corresponding to its depth value.
    The fill character encodes normalised depth.
    """
    grid_cols = len(profile)
    grid = [[EMPTY] * grid_cols for _ in range(grid_rows)]

    for col, depth in enumerate(profile):
        if np.isnan(depth) or depth <= 0:
            continue

        depth_clamped = min(depth, max_depth)
        # Which row does this depth correspond to?
        # row 0 = max_depth, row grid_rows-1 = 0
        fill_row = int((1.0 - depth_clamped / max_depth) * (grid_rows - 1))
        fill_row = max(0, min(fill_row, grid_rows - 1))

        normalised = depth_clamped / max_depth
        ch = _depth_to_char(normalised)

        # Fill from the depth row downward (toward the drone)
        for row in range(fill_row, grid_rows):
            grid[row][col] = ch

    return grid


def _add_drone_marker(grid):
    """
    Place the drone marker '^' at the bottom-centre of the grid.
    Returns the column index of the marker.
    """
    grid_rows = len(grid)
    grid_cols = len(grid[0])
    centre_col = grid_cols // 2
    grid[grid_rows - 1][centre_col] = DRONE_MARKER
    return centre_col


def _add_sonar_beam(grid, sonar_depth_m, max_depth, drone_col):
    """
    Draw a vertical sonar beam from the drone marker upward.

    The beam rises from the bottom row to the row matching sonar_depth_m,
    drawn as SONAR_BEAM '|' characters with SONAR_TIP 'S' at the top.
    """
    grid_rows = len(grid)

    sonar_clamped = min(sonar_depth_m, max_depth)
    tip_row = int((1.0 - sonar_clamped / max_depth) * (grid_rows - 1))
    tip_row = max(0, min(tip_row, grid_rows - 1))

    bottom_row = grid_rows - 1  # drone row

    # Draw beam upward from drone to tip (exclusive of drone marker row)
    for row in range(tip_row + 1, bottom_row):
        grid[row][drone_col] = SONAR_BEAM

    # Place tip marker
    grid[tip_row][drone_col] = SONAR_TIP


def _render_grid(grid, max_depth, sonar_depth_m=None, scale=None,
                 title="DEPTH MAP"):
    """
    Render a 2-D character grid to a string with axis labels.

    Left axis: distance in metres (approximate).
    Bottom:    drone reference marker.
    """
    grid_rows = len(grid)
    grid_cols = len(grid[0]) if grid_rows > 0 else 0

    lines = []
    lines.append("")
    lines.append("-" * (grid_cols + 12))
    header = "  " + title
    if sonar_depth_m is not None:
        header += "  (sonar={:.2f}m".format(sonar_depth_m)
        if scale is not None:
            header += "  scale={:.3f}".format(scale)
        else:
            header += "  scale=out-of-range"
        header += ")"
    lines.append(header)
    lines.append("  depth axis: 0=near  {}=far({:.1f}m)  drone=^  sonar=|S".format(
        len(FILLED_CHARS) - 1, max_depth))
    lines.append("")

    for row_idx, row in enumerate(grid):
        # Distance label for this row
        dist_m = max_depth * (1.0 - row_idx / max(grid_rows - 1, 1))
        label  = "{:4.1f}m | ".format(dist_m)
        lines.append(label + "".join(row))

    # Bottom axis: just mark the centre column
    bottom_pad   = " " * 9
    centre_col   = grid_cols // 2
    tick_row     = bottom_pad + " " * centre_col + "|"
    label_row    = bottom_pad + " " * (centre_col - 2) + "drone"
    lines.append(tick_row)
    lines.append(label_row)
    lines.append("")

    return "\n".join(lines)


# --------------------------------------------
# Public render functions
# --------------------------------------------

def render_depth_only(depth_map, max_depth=MAX_DEPTH_M,
                      grid_cols=GRID_COLS, grid_rows=GRID_ROWS,
                      title="DEPTH MAP (no sonar)"):
    """
    Render the raw MiDaS depth map as an ASCII grid with no sonar overlay.
    Returns the rendered string.
    """
    profile = _build_column_profile(depth_map, grid_cols)
    grid    = _profile_to_grid(profile, max_depth, grid_rows)
    _add_drone_marker(grid)
    return _render_grid(grid, max_depth, title=title)


def render_depth_with_sonar(depth_map, sonar_depth_m,
                             max_depth=MAX_DEPTH_M,
                             grid_cols=GRID_COLS, grid_rows=GRID_ROWS,
                             sonar_pixel=None,
                             title="DEPTH MAP + SONAR"):
    """
    Scale the depth map using the sonar reading, then render with sonar beam.

    Parameters:
        depth_map     : np.ndarray (H, W) - raw MiDaS output.
        sonar_depth_m : float - sonar distance in metres.
        max_depth     : float - metres that map to the top of the grid.
        sonar_pixel   : (row, col) aligned with sonar; defaults to image centre.

    Returns the rendered string.
    """
    scale = compute_scale(depth_map, sonar_depth_m, pixel=sonar_pixel)

    if scale is not None:
        scaled_map = depth_map * scale
    else:
        # Scale rejected - render unscaled with a warning
        scaled_map = depth_map.copy()

    profile    = _build_column_profile(scaled_map, grid_cols)
    grid       = _profile_to_grid(profile, max_depth, grid_rows)
    drone_col  = _add_drone_marker(grid)
    _add_sonar_beam(grid, sonar_depth_m, max_depth, drone_col)

    return _render_grid(grid, max_depth,
                        sonar_depth_m=sonar_depth_m,
                        scale=scale,
                        title=title)


# --------------------------------------------
# Depth map generators for testing
# --------------------------------------------

def make_coherent_depth_map(width=266, height=266, seed=0):
    """
    Generate a depth map with a smooth, wall-like obstacle structure.
    The scene has:
      - a near curved wall on the left third
      - a far flat background on the right two-thirds
      - a gentle ramp in the middle
    All values are raw MiDaS-style relative units (not metres).
    """
    rng = np.random.default_rng(seed)
    h, w = height, width
    depth = np.zeros((h, w), dtype=np.float32)

    # Background: far distance ~3.5 relative units
    depth[:, :] = rng.uniform(3.0, 4.0, (h, w))

    # Left third: close curved wall (~1.0-1.8 relative units)
    wall_end = w // 3
    for col in range(wall_end):
        centre_dist = 1.0 + 0.8 * (col / wall_end)
        depth[:, col] = rng.normal(centre_dist, 0.05, h).clip(0.5, 3.0)

    # Middle ramp: linear gradient from wall to background
    ramp_start = wall_end
    ramp_end   = 2 * w // 3
    for col in range(ramp_start, ramp_end):
        t = (col - ramp_start) / max(ramp_end - ramp_start - 1, 1)
        centre_dist = 1.8 + t * (3.5 - 1.8)
        depth[:, col] = rng.normal(centre_dist, 0.08, h).clip(0.5, 5.0)

    return depth


def make_random_depth_map(width=266, height=266, seed=None):
    """
    Generate a depth map with random rectangular obstacles at random distances.
    """
    rng  = np.random.default_rng(seed)
    base = rng.uniform(2.5, 5.0, (height, width)).astype(np.float32)

    n_obstacles = rng.integers(3, 7)
    for _ in range(n_obstacles):
        c0 = int(rng.uniform(0, width * 0.8))
        c1 = int(c0 + rng.uniform(width * 0.05, width * 0.3))
        r0 = int(rng.uniform(0, height * 0.8))
        r1 = int(r0 + rng.uniform(height * 0.1, height * 0.4))
        c1, r1 = min(c1, width), min(r1, height)
        dist = rng.uniform(0.6, 2.5)
        base[r0:r1, c0:c1] = rng.normal(dist, 0.05, (r1-r0, c1-c0)).clip(0.3, 5.0)

    return base


# --------------------------------------------
# Test functions
# --------------------------------------------

def test_coherent_scene():
    """
    Test 1: coherent wall scene.
    Prints the raw depth grid, then three sonar-scaled grids.
    The three sonar values are chosen to cover near, mid, and far readings
    so you can see the scale factor change the grid layout.
    """
    print("=" * 60)
    print("  TEST 1: COHERENT WALL SCENE")
    print("=" * 60)

    depth_map = make_coherent_depth_map(width=266, height=266, seed=0)

    print("  Depth map shape: {}  raw min={:.2f}  raw max={:.2f}".format(
        depth_map.shape, depth_map.min(), depth_map.max()))

    # --- Raw depth map (no sonar) ---
    print(render_depth_only(depth_map, title="DEPTH MAP (no sonar)"))

    # --- Three fixed sonar readings ---
    sonar_values = [1.2, 2.8, 4.5]   # near / mid / far
    for sv in sonar_values:
        print(render_depth_with_sonar(
            depth_map, sv,
            title="DEPTH MAP + SONAR"))


def test_interactive():
    """
    Test 2: interactive dialogue.
    Asks the user to type a sonar value (in metres) and renders the result.
    Type 'q' or press Enter with no value to quit.
    """
    print("=" * 60)
    print("  TEST 2: INTERACTIVE SONAR INPUT")
    print("  Using coherent wall scene as the base depth map.")
    print("  Type a sonar distance in metres and press Enter.")
    print("  Press Enter with no input to quit.")
    print("=" * 60)

    depth_map = make_coherent_depth_map(width=266, height=266, seed=0)

    while True:
        raw = input("\n  Sonar distance (metres) > ").strip()

        if raw == '' or raw.lower() == 'q':
            print("  Exiting interactive test.")
            break

        try:
            sonar_m = float(raw)
        except ValueError:
            print("  Invalid input - please enter a number.")
            continue

        if sonar_m <= 0:
            print("  Sonar value must be positive.")
            continue

        print(render_depth_with_sonar(
            depth_map, sonar_m,
            title="DEPTH MAP + SONAR (input={:.2f}m)".format(sonar_m)))


# --------------------------------------------
# Entry point
# --------------------------------------------

if __name__ == "__main__":
    test_coherent_scene()
    test_interactive()
