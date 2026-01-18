import math
from collections import defaultdict

class HeuristicProjection:
    #Generates a conservative heuristic cost map from a StaticOccupancyGrid.

    def __init__(
        self,
        static_grid,
        bounds,
        dilation_radius=3,
        dilation_cost=12.0,

        directional_radius=3,
        directional_cost=10.0,

        height_variance_thresh=0.6,
        height_cost=8.0,

        halo_radius=4,
        halo_cost=6.0
    ):
        #variables necessary for map expansions
        self.static_grid = static_grid
        self.bounds = bounds
        self.heuristic_cost = defaultdict(float)

        # Tunables
        self.dilation_radius = dilation_radius
        self.dilation_cost = dilation_cost

        self.directional_radius = directional_radius
        self.directional_cost = directional_cost

        self.height_variance_thresh = height_variance_thresh
        self.height_cost = height_cost

        self.halo_radius = halo_radius
        self.halo_cost = halo_cost
    #helper utilities
    def _neighbors(self, x, y, r=1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx == 0 and dy == 0:
                    continue
                yield x + dx, y + dy
    def _distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
    #------------------------------
    #heuristic expansion algorithms
    #------------------------------
    #this one is kinda pointless ngl, might just comment it out for now
    '''
    def _apply_morphological_dilation(self):
        for (x, y) in self.static_grid.grid:
            for nx, ny in self._neighbors(x, y, self.dilation_radius):

                #BOUNDS CHECK (planning bounds)
                if not self.bounds.within_planning_bounds_2d(nx, ny):
                    continue

                d = self._distance((x, y), (nx, ny))
                if d > self.dilation_radius:
                    continue

                cost = self.dilation_cost * (1 - d / self.dilation_radius)
                self.heuristic_cost[(nx, ny)] += max(cost, 0)'''
    #(needs fine tuning, TOO conservative)
    def _apply_directional_extension(self):
        for (x, y), cell in self.static_grid.grid.items():

            # ---------------------------
            # 1. Detect wall orientation
            # ---------------------------

            # Cardinal neighbors
            left  = (x - 1, y) in self.static_grid.grid
            right = (x + 1, y) in self.static_grid.grid
            up    = (x, y + 1) in self.static_grid.grid
            down  = (x, y - 1) in self.static_grid.grid

            # Diagonal neighbors (tolerance for stair-step walls)
            diag_count = sum([
                (x + 1, y + 1) in self.static_grid.grid,
                (x - 1, y - 1) in self.static_grid.grid,
                (x + 1, y - 1) in self.static_grid.grid,
                (x - 1, y + 1) in self.static_grid.grid,
            ])

            vertical_alignment = up or down
            horizontal_alignment = left or right

            # Diagonal-only support (weak evidence)
            if not vertical_alignment and not horizontal_alignment and diag_count >= 2:
                # Infer both weakly; cost decay will limit impact
                vertical_alignment = True
                horizontal_alignment = True

            # If no directional evidence at all, skip
            if not vertical_alignment and not horizontal_alignment:
                continue

            # ------------------------------------
            # 2. Choose orthogonal extension axes
            # ------------------------------------

            extension_dirs = []

            # Wall is vertical → extend horizontally
            if vertical_alignment:
                extension_dirs.extend([(1, 0), (-1, 0)])

            # Wall is horizontal → extend vertically
            if horizontal_alignment:
                extension_dirs.extend([(0, 1), (0, -1)])

            # ------------------------------------
            # 3. Apply directional extension
            # ------------------------------------

            for dx, dy in extension_dirs:
                for i in range(1, self.directional_radius + 1):
                    nx = x + dx * i
                    ny = y + dy * i

                    # Stop if outside planning bounds
                    if not self.bounds.within_planning_bounds_2d(nx, ny):
                        break

                    # Stop if we hit a static obstacle
                    if (nx, ny) in self.static_grid.grid:
                        break

                    cost = self.directional_cost * (1 - i / self.directional_radius)
                    if cost <= 0:
                        break

                    self.heuristic_cost[(nx, ny)] += cost

    #assumes consistent height metrics in neighboring cells
    def _apply_height_consistency(self):
        for (x, y), cell in self.static_grid.grid.items():
            z_range = cell["z_max"] - cell["z_min"]

            for nx, ny in self._neighbors(x, y):
                if (nx, ny) not in self.static_grid.grid:
                    continue
                if not self.bounds.within_planning_bounds_2d(nx, ny):
                    continue

                neighbor = self.static_grid.grid[(nx, ny)]
                nz_range = neighbor["z_max"] - neighbor["z_min"]

                if abs(z_range - nz_range) <= self.height_variance_thresh:
                    for ex, ey in self._neighbors(nx, ny, 1):
                        if (ex, ey) in self.static_grid.grid:
                            continue

                        self.heuristic_cost[(ex, ey)] += self.height_cost
    #essentially creates a deteriorating confidence gradient around identified cells
    def _apply_uncertainty_halo(self):
        for (x, y) in self.static_grid.grid:
            for nx, ny in self._neighbors(x, y, self.halo_radius):
                if (nx, ny) in self.static_grid.grid:
                    continue
                if not self.bounds.within_planning_bounds_2d(nx, ny):
                    continue

                d = self._distance((x, y), (nx, ny))
                if d > self.halo_radius:
                    continue

                cost = self.halo_cost * (1 - d / self.halo_radius)
                self.heuristic_cost[(nx, ny)] += max(cost, 0)
    #--------------
    #build function
    #--------------
    def build(self):
        self.heuristic_cost.clear()
        #self._apply_morphological_dilation()
        self._apply_directional_extension()
        self._apply_height_consistency()
        self._apply_uncertainty_halo()

        return dict(self.heuristic_cost)
