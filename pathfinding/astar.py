
"""
pathfinding.py
--------------
A* pathfinding implementation using a ConvergedProjection cost map.
"""

import heapq
import math

class AStarPlanner:
    def __init__(self, cost_map, bounds=None, diagonal_cost=1.414, straight_cost=1.0):
        """
        cost_map: dict[(x, y)] -> float, from ConvergedProjection
        bounds: optional Bounds2D object
        """
        self.cost_map = cost_map
        self.bounds = bounds
        self.diagonal_cost = diagonal_cost
        self.straight_cost = straight_cost

    def _in_bounds(self, x, y):
        if self.bounds:
            return self.bounds.within_planning_bounds_2d(x, y)
        return True

    def _neighbors(self, x, y):
        # 8-connected grid
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),  # straight
            (1, 1), (-1, 1), (1, -1), (-1, -1)  # diagonal
        ]
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if not self._in_bounds(nx, ny):
                continue
            yield nx, ny

    def _movement_cost(self, from_node, to_node):
        x1, y1 = from_node
        x2, y2 = to_node
        # Determine if diagonal
        if abs(x2 - x1) + abs(y2 - y1) == 2:
            base_cost = self.diagonal_cost
        else:
            base_cost = self.straight_cost

        # Add cell cost if it exists in cost_map
        extra_cost = self.cost_map.get((x2, y2), 0)
        return base_cost + extra_cost

    def _heuristic(self, node, goal):
        # Euclidean distance
        x1, y1 = node
        x2, y2 = goal
        return math.hypot(x2 - x1, y2 - y1)

    def plan(self, start, goal):
        """
        start, goal: tuples (x, y)
        Returns path as list of (x, y) from start to goal, or empty list if impossible
        """
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for neighbor in self._neighbors(*current):
                tentative_g = g_score[current] + self._movement_cost(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # No path found
        return []

# =========================
# EXAMPLE USAGE
# =========================
if __name__ == "__main__":
    from converged import ConvergedProjection, StaticOccupancyGrid, Bounds2D
    # Setup minimal test map
    bounds = Bounds2D(-5, 5, -5, 5)
    static_grid = StaticOccupancyGrid()
    static_grid.add_cell(0, 0)  # obstacle in center

    heuristic_costs = {}  # empty for test
    conv_proj = ConvergedProjection(static_grid, heuristic_costs, bounds)
    cost_map = conv_proj.build()

    planner = AStarPlanner(cost_map, bounds=bounds)
    path = planner.plan((-4, -4), (4, 4))
    print("Planned path:", path)
