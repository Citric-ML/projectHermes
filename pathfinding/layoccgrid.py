import math
from bounds import Bounds
from collections import defaultdict
class StaticOccupancyGrid:
    """
    Long-term, high-confidence 2.5D occupancy grid.
    Stores only persistent structures.
    """
    def __init__(self):
        # key: (x, y) → value: dict
        self.grid = {}

    def add_or_update_cell(self, x, y, z_min, z_max, confidence):
        if (x, y) not in self.grid:
            self.grid[(x, y)] = {
                "z_min": z_min,
                "z_max": z_max,
                "confidence": confidence
            }
        else:
            cell = self.grid[(x, y)]
            cell["z_min"] = min(cell["z_min"], z_min)
            cell["z_max"] = max(cell["z_max"], z_max)
            cell["confidence"] = min(1.0, max(cell["confidence"], confidence))

    def is_occupied(self, x, y):
        return (x, y) in self.grid

#this is the dynamic occupancy grid, since it stores more recent local information
class LayeredOccupancyGrid:
    def __init__(self, cell_size=0.5):
        #cell_size: meters per grid cell
        self.cell_size = cell_size

        #each cell has occupancy 0-1, z_min, confidence(of object permanence), z_max, observation count
        self.grid = defaultdict(lambda: {
            "occ": 0.0,
            "z_min": None,
            "confidence": 0,
            "z_max": None,
            "count": 0
        })

    def _world_to_grid(self, x, y):
        #convert rw coords to grid indices
        gx = int(math.floor(x / self.cell_size))
        gy = int(math.floor(y / self.cell_size))
        return gx, gy

    def _direction_to_unit_vector(self, yaw_deg, pitch_deg):
        #Convert yaw/pitch to a unit direction vector (raycasting)
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)

        dx = math.cos(pitch) * math.cos(yaw)
        dy = math.cos(pitch) * math.sin(yaw)
        dz = math.sin(pitch)

        return dx, dy, dz

    def update(self, drone_pos, direction_deg, depth):
        """
        drone_pos: (x, y, z) in meters
        direction_deg: (yaw, pitch) in degrees
        depth: meters
        this is basically just raycasting
        """
        x_d, y_d, z_d = drone_pos
        yaw, pitch = direction_deg

        # Direction vector
        dx, dy, dz = self._direction_to_unit_vector(yaw, pitch)

        # Project hit point (implicit ray casting)
        x_hit = x_d + dx * depth
        y_hit = y_d + dy * depth
        z_hit = z_d + dz * depth
        if not Bounds.within_hard_bounds(x_hit, y_hit, z_hit):
            return

        Bounds.update_observed_bounds(x_hit, y_hit, z_hit)

        # Determine grid cell
        gx, gy = self._world_to_grid(x_hit, y_hit)
        cell = self.grid[(gx, gy)]

        # Update occupancy w/Bayesian-ish increment
        cell["occ"] = min(1.0, cell["occ"] + 0.2)
        cell["count"] += 1

        # Update confidence (permanence belief)
        cell["confidence"] = min(1.0, cell["confidence"] + 0.1)


        # Update vertical bounds
        if cell["z_min"] is None or z_hit < cell["z_min"]:
            cell["z_min"] = z_hit
        if cell["z_max"] is None or z_hit > cell["z_max"]:
            cell["z_max"] = z_hit

    def get_cell(self, x, y):
        #Query cell by world coordinates
        gx, gy = self._world_to_grid(x, y)
        return self.grid.get((gx, gy), None)

    def debug_print(self):
        #Print all occupied cells
        for (gx, gy), cell in self.grid.items():
            if cell["occ"] > 0:
                print(f"Cell ({gx}, {gy}): occ={cell['occ']:.2f}, "
                      f"z=[{cell['z_min']:.2f}, {cell['z_max']:.2f}]")
                
grid = LayeredOccupancyGrid(cell_size=0.5)

def promote_static_cells(layered_grid,
                         static_grid,
                         confidence_threshold=0.8,
                         min_observations=5,
                         max_height_variance=0.3):

    to_promote = []

    for (x, y), cell in layered_grid.grid.items():
        if cell["z_min"] is None or cell["z_max"] is None:
            continue

        height_variance = cell["z_max"] - cell["z_min"]

        if (
            cell["confidence"] >= confidence_threshold
            and cell["count"] >= min_observations
            and height_variance <= max_height_variance
        ):
            to_promote.append((x, y))

    for (x, y) in to_promote:
        cell = layered_grid.grid[(x, y)]
        static_grid.add_or_update_cell(
            x, y,
            cell["z_min"],
            cell["z_max"],
            cell["confidence"]
        )
        del layered_grid.grid[(x, y)]

#combines the two layered occupancy grids to form a combined weight projection for A*
class CombinedOccupancyProjection:
    def __init__(self, static_grid, dynamic_grid,
                 static_weight=1000.0,
                 dynamic_weight=20.0,
                 uncertainty_weight=5.0):
        self.static_grid = static_grid
        self.dynamic_grid = dynamic_grid
        self.static_weight = static_weight
        self.dynamic_weight = dynamic_weight
        self.uncertainty_weight = uncertainty_weight

    def get_cost_map(self):
        """
        Returns a dictionary mapping (x, y) → combined cost.
        """
        cost_map = {}

        # Include all cells from static and dynamic grids
        keys = set(self.static_grid.grid.keys()).union(self.dynamic_grid.grid.keys())

        for key in keys:
            cost = 0.0

            # Static cost
            if key in self.static_grid.grid:
                cell = self.static_grid.grid[key]
                cost += self.static_weight * cell["confidence"]

            # Dynamic cost
            if key in self.dynamic_grid.grid:
                cell = self.dynamic_grid.grid[key]
                # Dynamic cost scaled by occupancy and 1-confidence
                cost += self.dynamic_weight * cell["occ"]
                cost += self.uncertainty_weight * (1.0 - cell["confidence"])

            cost_map[key] = cost

        return cost_map


'''
---TESTING---(commented out bc it doesn't need to run)

import random
import time
import math

def test_random_projection(num_observations=50, repeat_for_static=5):
    # Initialize grids
    dynamic_grid = LayeredOccupancyGrid(cell_size=1.0)
    static_grid = StaticOccupancyGrid()

    start_time = time.time()

    # Simulate drone observations
    for i in range(num_observations):
        # Random drone position (x, y, z)
        x_d = random.uniform(0, 10)
        y_d = random.uniform(0, 10)
        z_d = random.uniform(0, 2)

        # Random direction (yaw, pitch)
        yaw = random.uniform(0, 360)
        pitch = random.uniform(-10, 10)

        # Random depth 1–5 meters
        depth = random.uniform(1, 5)

        # Update dynamic grid
        dynamic_grid.update(
            drone_pos=(x_d, y_d, z_d),
            direction_deg=(yaw, pitch),
            depth=depth
        )

        # Repeat some observations to simulate permanence
        if i % repeat_for_static == 0:
            dynamic_grid.update(
                drone_pos=(x_d, y_d, z_d),
                direction_deg=(yaw, pitch),
                depth=depth
            )

        # Apply dynamic decay (simulate time passing)
        current_time = time.time() - start_time
        dynamic_grid_decay_rate = 0.01  # can tweak
        dynamic_grid.forget_threshold = 0.2
        # we'll implement decay via the observation count vs confidence, simplified here
        for cell in dynamic_grid.grid.values():
            cell["confidence"] = max(0.0, cell["confidence"] - dynamic_grid_decay_rate)

    # Promote cells to static with higher thresholds
    promote_static_cells(dynamic_grid, static_grid,
                         confidence_threshold=0.8,  # realistic for static
                         min_observations=5,
                         max_height_variance=1.0)

    # Generate combined cost map
    projection = CombinedOccupancyProjection(static_grid, dynamic_grid)
    cost_map = projection.get_cost_map()

    # Print dynamic grid
    print("\n--- DYNAMIC GRID ---")
    for (x, y), cell in dynamic_grid.grid.items():
        print(f"Cell ({x},{y}): occ={cell['occ']:.2f}, confidence={cell['confidence']:.2f}, z=[{cell['z_min']:.2f},{cell['z_max']:.2f}]")

    # Print static grid
    print("\n--- STATIC GRID ---")
    for (x, y), cell in static_grid.grid.items():
        print(f"Cell ({x},{y}): confidence={cell['confidence']:.2f}, z=[{cell['z_min']:.2f},{cell['z_max']:.2f}]")

    # Print combined cost map
    print("\n--- COMBINED COST MAP ---")
    for (x, y), cost in cost_map.items():
        print(f"Cell ({x},{y}): cost={cost:.2f}")
    # 2D ASCII visualization
    print("\n--- 2D GRID VISUALIZATION ---")
    # Determine bounds
    all_coords = list(dynamic_grid.grid.keys()) + list(static_grid.grid.keys())
    if not all_coords:
        print("No cells to display")
        return

    xs, ys = zip(*all_coords)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    for y in range(max_y, min_y - 1, -1):
        row = ""
        for x in range(min_x, max_x + 1):
            if (x, y) in static_grid.grid:
                row += "S"
            elif (x, y) in dynamic_grid.grid:
                row += "D"
            elif (x, y) in cost_map:
                row += "*"
            else:
                row += "."
        print(row)
test_random_projection()'''
