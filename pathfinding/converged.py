"""
Converged Projection + A* Test
------------------------------
Merges static and heuristic projections with bounds checking,
then runs A* pathfinding from one side to the other.
"""

from collections import defaultdict
import heapq
import math

# =========================
# CONFIG
# =========================
GRID_MIN_X = -10
GRID_MAX_X = 10
GRID_MIN_Y = -10
GRID_MAX_Y = 10
STATIC_COST = 200.0

# =========================
# BOUNDS CLASS
# =========================
class Bounds2D:
    def __init__(self, min_x, max_x, min_y, max_y):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

    def within_planning_bounds_2d(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

bounds = Bounds2D(GRID_MIN_X, GRID_MAX_X, GRID_MIN_Y, GRID_MAX_Y)

# =========================
# STATIC GRID (TEST STUB)
# =========================
class StaticOccupancyGrid:
    def __init__(self):
        self.grid = {}

    def add_cell(self, x, y, z_min=0.0, z_max=2.0, confidence=1.0):
        self.grid[(x, y)] = {"z_min": z_min, "z_max": z_max, "confidence": confidence}

def build_plus_corridor(static_grid, arm_length=6):
    for i in range(-arm_length, arm_length + 1):
        static_grid.add_cell(i, 0)
        static_grid.add_cell(0, i)

# =========================
# HEURISTIC IMPORT
# =========================
from heuristic import HeuristicProjection

# =========================
# CONVERGED PROJECTION
# =========================
class ConvergedProjection:
    def __init__(self, static_grid, heuristic_costs, bounds):
        self.static_grid = static_grid
        self.heuristic_costs = heuristic_costs
        self.bounds = bounds
        self.cost_map = {}

    def build(self):
        self.cost_map.clear()
        for (x, y) in self.static_grid.grid:
            if not self.bounds.within_planning_bounds_2d(x, y):
                continue
            self.cost_map[(x, y)] = STATIC_COST

        for (x, y), cost in self.heuristic_costs.items():
            if not self.bounds.within_planning_bounds_2d(x, y):
                continue
            if (x, y) in self.cost_map:
                continue
            self.cost_map[(x, y)] = cost

        return self.cost_map

# =========================
# A* PATHFINDER
# =========================

from astar import AStarPlanner as Astar

# =========================
# VISUALIZATION
# =========================
def visualize(cost_map, path=None):
    for y in range(GRID_MAX_Y, GRID_MIN_Y - 1, -1):
        row = ""
        for x in range(GRID_MIN_X, GRID_MAX_X + 1):
            if path and (x, y) in path:
                row += "+"
            elif (x, y) in cost_map:
                if cost_map[(x, y)] >= STATIC_COST:
                    row += "#"
                else:
                    row += "H"
            else:
                row += "."
        print(row)

# =========================
# MAIN TEST
# =========================
if __name__ == "__main__":
    static_grid = StaticOccupancyGrid()
    build_plus_corridor(static_grid, arm_length=6)

    heuristic = HeuristicProjection(static_grid, bounds=bounds)
    heuristic_costs = heuristic.build()

    converged = ConvergedProjection(static_grid, heuristic_costs, bounds)
    cost_map = converged.build()

    planner = Astar(cost_map, bounds=bounds)
    start = (-6, -6)
    goal = (6, 6)
    path = planner.plan(start, goal)

    print("\n=== CONVERGED COST MAP WITH PATH ===\n")
    visualize(cost_map, path=path)
    print("\nPath coordinates:", path)
