"""
SLAM Pipeline Visualizer
========================
Visualizes the four occupancy projections as ASCII grids:
  1. Dynamic grid   - confidence scores of recently observed cells
  2. Static grid    - promoted high-confidence persistent cells
  3. Heuristic grid - newly added heuristic expansion cells only
  4. Converged grid - merged cost map fed to A*

Test harness: accepts a MiDaS-style 2D numpy depth map, projects it
into the dynamic grid, then randomly promotes a subset of cells to
the static grid.
"""

import math
import random
import numpy as np
from collections import defaultdict

# --------------------------------------------
# Minimal Bounds stub (replaces Grid_building.bounds)
# --------------------------------------------
class _Bounds:
    HARD_MIN_X, HARD_MAX_X = -30.0, 30.0
    HARD_MIN_Y, HARD_MAX_Y = -30.0, 30.0
    HARD_MIN_Z, HARD_MAX_Z = -5.0,  20.0

    _obs_min_x = _obs_min_y = _obs_min_z =  1e9
    _obs_max_x = _obs_max_y = _obs_max_z = -1e9

    @classmethod
    def within_hard_bounds(cls, x, y, z):
        return (cls.HARD_MIN_X <= x <= cls.HARD_MAX_X and
                cls.HARD_MIN_Y <= y <= cls.HARD_MAX_Y and
                cls.HARD_MIN_Z <= z <= cls.HARD_MAX_Z)

    @classmethod
    def update_observed_bounds(cls, x, y, z):
        cls._obs_min_x = min(cls._obs_min_x, x)
        cls._obs_max_x = max(cls._obs_max_x, x)
        cls._obs_min_y = min(cls._obs_min_y, y)
        cls._obs_max_y = max(cls._obs_max_y, y)
        cls._obs_min_z = min(cls._obs_min_z, z)
        cls._obs_max_z = max(cls._obs_max_z, z)

    @classmethod
    def within_planning_bounds_2d(cls, x, y):
        return (cls.HARD_MIN_X <= x <= cls.HARD_MAX_X and
                cls.HARD_MIN_Y <= y <= cls.HARD_MAX_Y)

Bounds = _Bounds()

# --------------------------------------------
# Grid classes (self-contained, no file imports)
# --------------------------------------------

class StaticOccupancyGrid:
    def __init__(self):
        self.grid = {}

    def add_or_update_cell(self, x, y, z_min, z_max, confidence):
        if (x, y) not in self.grid:
            self.grid[(x, y)] = {"z_min": z_min, "z_max": z_max, "confidence": confidence}
        else:
            cell = self.grid[(x, y)]
            cell["z_min"] = min(cell["z_min"], z_min)
            cell["z_max"] = max(cell["z_max"], z_max)
            cell["confidence"] = min(1.0, max(cell["confidence"], confidence))

    def is_occupied(self, x, y):
        return (x, y) in self.grid


class LayeredOccupancyGrid:
    def __init__(self, cell_size=0.5):
        self.cell_size = cell_size
        self.grid = defaultdict(lambda: {
            "occ": 0.0, "z_min": None, "confidence": 0.0,
            "z_max": None, "count": 0
        })

    def _world_to_grid(self, x, y):
        return int(math.floor(x / self.cell_size)), int(math.floor(y / self.cell_size))

    def update_world_hit(self, x_hit, y_hit, z_hit):
        if not _Bounds.within_hard_bounds(x_hit, y_hit, z_hit):
            return
        _Bounds.update_observed_bounds(x_hit, y_hit, z_hit)

        gx, gy = self._world_to_grid(x_hit, y_hit)
        cell = self.grid[(gx, gy)]
        cell["occ"]        = min(1.0, cell["occ"] + 0.2)
        cell["count"]     += 1
        cell["confidence"] = min(1.0, cell["confidence"] + 0.1)

        if cell["z_min"] is None or z_hit < cell["z_min"]:
            cell["z_min"] = z_hit
        if cell["z_max"] is None or z_hit > cell["z_max"]:
            cell["z_max"] = z_hit

    def decay(self, decay_rate=0.02):
        to_delete = [k for k, c in self.grid.items() if c["confidence"] <= decay_rate]
        for k in to_delete:
            del self.grid[k]
        for cell in self.grid.values():
            cell["confidence"] = max(0.0, cell["confidence"] - decay_rate)


class HeuristicProjection:
    """Stripped-down version; only directional extension enabled (matches heuristic.py)."""

    def __init__(self, static_grid, bounds,
                 directional_radius=3, directional_cost=10.0):
        self.static_grid        = static_grid
        self.bounds             = bounds
        self.heuristic_cost     = defaultdict(float)
        self.directional_radius = directional_radius
        self.directional_cost   = directional_cost

    def _neighbors(self, x, y, r=1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx == 0 and dy == 0:
                    continue
                yield x + dx, y + dy

    def _apply_directional_extension(self):
        for (x, y) in self.static_grid.grid:
            left  = (x - 1, y) in self.static_grid.grid
            right = (x + 1, y) in self.static_grid.grid
            up    = (x, y + 1) in self.static_grid.grid
            down  = (x, y - 1) in self.static_grid.grid

            diag_count = sum([
                (x+1, y+1) in self.static_grid.grid,
                (x-1, y-1) in self.static_grid.grid,
                (x+1, y-1) in self.static_grid.grid,
                (x-1, y+1) in self.static_grid.grid,
            ])

            vertical   = up or down
            horizontal = left or right
            if not vertical and not horizontal and diag_count >= 2:
                vertical = horizontal = True
            if not vertical and not horizontal:
                continue

            dirs = []
            if vertical:   dirs.extend([(1, 0), (-1, 0)])
            if horizontal: dirs.extend([(0, 1), (0, -1)])

            for dx, dy in dirs:
                for i in range(1, self.directional_radius + 1):
                    nx, ny = x + dx * i, y + dy * i
                    if not self.bounds.within_planning_bounds_2d(nx, ny):
                        break
                    if (nx, ny) in self.static_grid.grid:
                        break
                    cost = self.directional_cost * (1 - i / self.directional_radius)
                    if cost <= 0:
                        break
                    self.heuristic_cost[(nx, ny)] += cost

    def build(self):
        self.heuristic_cost.clear()
        self._apply_directional_extension()
        return dict(self.heuristic_cost)


STATIC_COST = 200.0

class ConvergedProjection:
    def __init__(self, static_grid, heuristic_costs, bounds):
        self.static_grid     = static_grid
        self.heuristic_costs = heuristic_costs
        self.bounds          = bounds
        self.cost_map        = {}

    def build(self):
        self.cost_map.clear()
        for (x, y) in self.static_grid.grid:
            if self.bounds.within_planning_bounds_2d(x, y):
                self.cost_map[(x, y)] = STATIC_COST
        for (x, y), cost in self.heuristic_costs.items():
            if self.bounds.within_planning_bounds_2d(x, y):
                if (x, y) not in self.cost_map:
                    self.cost_map[(x, y)] = cost
        return self.cost_map


# --------------------------------------------
# Depth-map -> dynamic grid  (from main_loop.py)
# --------------------------------------------

def project_depth_to_grid(depth_map, drone_pose, dynamic_grid, sample_step=4):
    """
    Projects a MiDaS-style 2D depth array into the LayeredOccupancyGrid.

    depth_map  : np.ndarray (H, W) - relative depth values, already scaled to metres
    drone_pose : dict with keys x, y, z, yaw  (yaw in degrees)
    """
    if depth_map is None:
        return

    h, w = depth_map.shape
    cx, cy = w / 2, h / 2

    h_fov = np.radians(60.0)
    v_fov = np.radians(45.0)

    yaw     = np.radians(drone_pose.get("yaw", 0.0))
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    for row in range(0, h, sample_step):
        for col in range(0, w, sample_step):
            depth = depth_map[row, col]
            if not np.isfinite(depth) or depth <= 0:
                continue

            x_angle = ((col - cx) / cx) * (h_fov / 2)
            y_angle = ((row - cy) / cy) * (v_fov / 2)

            dx_cam = np.cos(y_angle) * np.sin(x_angle)
            dy_cam = np.sin(y_angle)
            dz_cam = np.cos(y_angle) * np.cos(x_angle)

            x_world = drone_pose["x"] + (dx_cam * cos_yaw - dz_cam * sin_yaw) * depth
            y_world = drone_pose["y"] +  dy_cam * depth
            z_world = drone_pose["z"] + (dx_cam * sin_yaw + dz_cam * cos_yaw) * depth

            dynamic_grid.update_world_hit(x_world, y_world, z_world)


# --------------------------------------------
# ASCII Visualizer
# --------------------------------------------

EMPTY_CHAR = "."


def _compute_bounds(keys):
    if not keys:
        return 0, 0, 0, 0
    xs, ys = zip(*keys)
    return min(xs), max(xs), min(ys), max(ys)


def _pad_bounds(mn_x, mx_x, mn_y, mx_y, pad=1):
    return mn_x - pad, mx_x + pad, mn_y - pad, mx_y + pad


def _confidence_char(conf):
    """Map a 0-1 confidence value to a single digit character (0-9)."""
    if conf >= 0.9: return '9'
    if conf >= 0.8: return '8'
    if conf >= 0.7: return '7'
    if conf >= 0.6: return '6'
    if conf >= 0.5: return '5'
    if conf >= 0.4: return '4'
    if conf >= 0.3: return '3'
    if conf >= 0.2: return '2'
    if conf >= 0.1: return '1'
    return '0'


def _cost_char(cost):
    """Map a heuristic/converged cost to a printable character."""
    if cost >= STATIC_COST: return '#'
    if cost >= 15:          return 'H'
    if cost >= 10:          return 'h'
    if cost >= 5:           return '+'
    return ','


def _print_x_axis(mn_x, mx_x):
    """Prints a minimal x-axis tick and label row."""
    label_row = "       "
    tick_row  = "       "
    for x in range(mn_x, mx_x + 1):
        if x % 5 == 0:
            s = str(x)
            label_row += s.ljust(len(s))
            tick_row  += '|' + ' ' * (len(s) - 1)
        else:
            label_row += ' '
            tick_row  += ' '
    print(tick_row)
    print(label_row)


def print_dynamic_grid(dynamic_grid, title="DYNAMIC OCCUPANCY GRID"):
    """
    Prints dynamic grid cells as their confidence score (0-9).
    Empty cells are shown as EMPTY_CHAR.
    """
    keys = [k for k, c in dynamic_grid.grid.items() if c["occ"] > 0]
    print("\n" + "-" * 50)
    print("  " + title)
    if not keys:
        print("  (empty)")
        return

    mn_x, mx_x, mn_y, mx_y = _pad_bounds(*_compute_bounds(keys))

    print("  Char = confidence decile (0=low ... 9=high)  " + EMPTY_CHAR + "=empty")
    print("  Grid range x=[{},{}]  y=[{},{}]".format(mn_x, mx_x, mn_y, mx_y))
    print()

    for y in range(mx_y, mn_y - 1, -1):
        row_str = "  {:4d} | ".format(y)
        for x in range(mn_x, mx_x + 1):
            cell = dynamic_grid.grid.get((x, y))
            if cell and cell["occ"] > 0:
                row_str += _confidence_char(cell["confidence"])
            else:
                row_str += EMPTY_CHAR
        print(row_str)

    _print_x_axis(mn_x, mx_x)


def print_static_grid(static_grid, title="STATIC OCCUPANCY GRID"):
    """
    Prints static cells as their confidence score (0-9).
    Empty cells are shown as EMPTY_CHAR.
    """
    keys = list(static_grid.grid.keys())
    print("\n" + "-" * 50)
    print("  " + title)
    if not keys:
        print("  (empty)")
        return

    mn_x, mx_x, mn_y, mx_y = _pad_bounds(*_compute_bounds(keys))

    print("  Char = confidence decile (0=low ... 9=high)  " + EMPTY_CHAR + "=empty")
    print("  Grid range x=[{},{}]  y=[{},{}]".format(mn_x, mx_x, mn_y, mx_y))
    print()

    for y in range(mx_y, mn_y - 1, -1):
        row_str = "  {:4d} | ".format(y)
        for x in range(mn_x, mx_x + 1):
            cell = static_grid.grid.get((x, y))
            if cell:
                row_str += _confidence_char(cell["confidence"])
            else:
                row_str += EMPTY_CHAR
        print(row_str)

    _print_x_axis(mn_x, mx_x)


def print_heuristic_grid(static_grid, heuristic_costs,
                         title="HEURISTIC EXPANSION GRID"):
    """
    Shows only the newly added heuristic cells (not the static obstacles
    themselves), labelled by cost band.
    Legend:  #=static  H=cost>=15  h=cost>=10  +=cost>=5  ,=cost<5
    """
    static_keys    = set(static_grid.grid.keys())
    heuristic_only = {k: v for k, v in heuristic_costs.items()
                      if k not in static_keys}

    all_keys = static_keys | set(heuristic_only.keys())
    print("\n" + "-" * 50)
    print("  " + title)
    if not all_keys:
        print("  (empty)")
        return

    mn_x, mx_x, mn_y, mx_y = _pad_bounds(*_compute_bounds(list(all_keys)))

    print("  #=static  H=cost>=15  h=cost>=10  +=cost>=5  ,=cost<5  " + EMPTY_CHAR + "=empty")
    print("  Grid range x=[{},{}]  y=[{},{}]".format(mn_x, mx_x, mn_y, mx_y))
    print()

    for y in range(mx_y, mn_y - 1, -1):
        row_str = "  {:4d} | ".format(y)
        for x in range(mn_x, mx_x + 1):
            if (x, y) in static_keys:
                row_str += '#'
            elif (x, y) in heuristic_only:
                row_str += _cost_char(heuristic_only[(x, y)])
            else:
                row_str += EMPTY_CHAR
        print(row_str)

    _print_x_axis(mn_x, mx_x)


def print_converged_grid(cost_map, title="CONVERGED PROJECTION"):
    """
    Full merged cost map.
    Legend:  #=static(>=200)  H=cost>=15  h=cost>=10  +=cost>=5  ,=cost<5
    """
    print("\n" + "-" * 50)
    print("  " + title)
    if not cost_map:
        print("  (empty)")
        return

    mn_x, mx_x, mn_y, mx_y = _pad_bounds(*_compute_bounds(list(cost_map.keys())))

    print("  #=static(cost>={:.0f})  H=cost>=15  h=cost>=10  +=cost>=5  ,=cost<5  {}=free".format(
        STATIC_COST, EMPTY_CHAR))
    print("  Grid range x=[{},{}]  y=[{},{}]".format(mn_x, mx_x, mn_y, mx_y))
    print()

    for y in range(mx_y, mn_y - 1, -1):
        row_str = "  {:4d} | ".format(y)
        for x in range(mn_x, mx_x + 1):
            cost = cost_map.get((x, y))
            if cost is not None:
                row_str += _cost_char(cost)
            else:
                row_str += EMPTY_CHAR
        print(row_str)

    _print_x_axis(mn_x, mx_x)


# --------------------------------------------
# Test harness
# --------------------------------------------

def make_test_depth_map(width=64, height=48, seed=42):
    """
    Generates a synthetic MiDaS-style depth map (H x W float32 array).
    Values represent scaled depth in metres (0.5 - 6.0 m).
    A simple wall feature is embedded to give structure.
    """
    rng  = np.random.default_rng(seed)
    base = rng.uniform(2.0, 6.0, (height, width)).astype(np.float32)

    # Embed a wall-like region in the centre-left
    wall_col_start = int(width * 0.3)
    wall_col_end   = int(width * 0.45)
    base[:, wall_col_start:wall_col_end] = rng.uniform(
        0.8, 1.4, (height, wall_col_end - wall_col_start))

    # Embed a closer obstacle top-right
    base[0:int(height * 0.4), int(width * 0.65):] = rng.uniform(
        1.0, 2.0, (int(height * 0.4), width - int(width * 0.65)))

    return base


def randomly_promote_cells(dynamic_grid, static_grid,
                            promotion_fraction=0.25, seed=7):
    """
    Randomly selects a fraction of dynamic cells and force-promotes them
    to the static grid with a high confidence score, simulating repeated
    observations of persistent structures.
    """
    rng = random.Random(seed)
    candidates = [
        (k, c) for k, c in dynamic_grid.grid.items()
        if c["z_min"] is not None and c["z_max"] is not None
    ]
    n_promote = max(1, int(len(candidates) * promotion_fraction))
    chosen    = rng.sample(candidates, min(n_promote, len(candidates)))

    for (gx, gy), cell in chosen:
        high_conf = rng.uniform(0.82, 1.0)
        static_grid.add_or_update_cell(
            gx, gy,
            cell["z_min"], cell["z_max"],
            high_conf
        )
        # Remove from dynamic so it does not appear in both grids
        del dynamic_grid.grid[(gx, gy)]

    return len(chosen)


def run_test():
    print("=" * 50)
    print("  SLAM PIPELINE VISUALIZER - TEST RUN")
    print("=" * 50)

    # 1. Build depth map
    print("\n[1] Generating synthetic MiDaS depth map (64x48) ...")
    depth_map = make_test_depth_map(width=64, height=48, seed=42)
    print("    Shape: {}  min={:.2f}m  max={:.2f}m".format(
        depth_map.shape, depth_map.min(), depth_map.max()))

    # 2. Project into dynamic grid
    print("\n[2] Projecting depth map into LayeredOccupancyGrid ...")
    drone_pose   = {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0.0}
    dynamic_grid = LayeredOccupancyGrid(cell_size=0.5)
    project_depth_to_grid(depth_map, drone_pose, dynamic_grid, sample_step=2)
    n_dynamic = sum(1 for c in dynamic_grid.grid.values() if c["occ"] > 0)
    print("    Occupied dynamic cells: {}".format(n_dynamic))

    # 3. Promote subset to static
    print("\n[3] Randomly promoting ~25% of dynamic cells to StaticOccupancyGrid ...")
    static_grid = StaticOccupancyGrid()
    n_promoted  = randomly_promote_cells(dynamic_grid, static_grid,
                                         promotion_fraction=0.25, seed=7)
    n_remaining = sum(1 for c in dynamic_grid.grid.values() if c["occ"] > 0)
    print("    Promoted cells: {}  |  Remaining dynamic: {}".format(
        n_promoted, n_remaining))

    # 4. Heuristic projection
    print("\n[4] Building HeuristicProjection from static grid ...")
    heuristic       = HeuristicProjection(static_grid, bounds=Bounds)
    heuristic_costs = heuristic.build()
    heuristic_only  = {k: v for k, v in heuristic_costs.items()
                       if k not in static_grid.grid}
    print("    Heuristic expansion cells: {}".format(len(heuristic_only)))

    # 5. Converged projection
    print("\n[5] Building ConvergedProjection ...")
    converged = ConvergedProjection(static_grid, heuristic_costs, bounds=Bounds)
    cost_map  = converged.build()
    print("    Total converged cells: {}".format(len(cost_map)))

    # 6. Print all four grids
    print_dynamic_grid(dynamic_grid)
    print_static_grid(static_grid)
    print_heuristic_grid(static_grid, heuristic_costs)
    print_converged_grid(cost_map)

    print("\n" + "-" * 50)
    print("  Done.\n")


if __name__ == "__main__":
    run_test()
