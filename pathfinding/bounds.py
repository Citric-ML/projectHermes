class Bounds:
    """
    Manages spatial bounds for mapping, heuristics, and planning.

    Bounds hierarchy:
    1. Hard bounds     → absolute representational limits
    2. Observed bounds → expanded by sensor observations
    3. Planning bounds → subset used by planners/heuristics
    """

    def __init__(
        self,
        hard_min=(-100, -100, -10),
        hard_max=(100, 100, 10),
        planning_margin=2
    ):
        # Hard bounds (never violated)
        self.hard_min = list(hard_min)
        self.hard_max = list(hard_max)

        # Observed bounds (expand with evidence)
        self.obs_min = [float("inf")] * 3
        self.obs_max = [float("-inf")] * 3

        # Planning bounds (derived)
        self.planning_margin = planning_margin

    # -----------------------------
    # Hard bounds checks
    # -----------------------------
    def within_hard_bounds(self, x, y, z):
        return (
            self.hard_min[0] <= x <= self.hard_max[0]
            and self.hard_min[1] <= y <= self.hard_max[1]
            and self.hard_min[2] <= z <= self.hard_max[2]
        )

    # -----------------------------
    # Observed bounds handling
    # -----------------------------
    def update_observed_bounds(self, x, y, z):
        self.obs_min[0] = min(self.obs_min[0], x)
        self.obs_min[1] = min(self.obs_min[1], y)
        self.obs_min[2] = min(self.obs_min[2], z)

        self.obs_max[0] = max(self.obs_max[0], x)
        self.obs_max[1] = max(self.obs_max[1], y)
        self.obs_max[2] = max(self.obs_max[2], z)

    def has_observed_data(self):
        return self.obs_min[0] != float("inf")

    # -----------------------------
    # Planning bounds
    # -----------------------------
    def get_planning_bounds(self):
        """
        Returns planning bounds expanded from observed bounds.
        """
        if not self.has_observed_data():
            return None

        min_x = self.obs_min[0] - self.planning_margin
        min_y = self.obs_min[1] - self.planning_margin
        min_z = self.obs_min[2] - self.planning_margin

        max_x = self.obs_max[0] + self.planning_margin
        max_y = self.obs_max[1] + self.planning_margin
        max_z = self.obs_max[2] + self.planning_margin

        return (min_x, min_y, min_z), (max_x, max_y, max_z)

    def within_planning_bounds(self, x, y, z):
        bounds = self.get_planning_bounds()
        if bounds is None:
            return False

        (min_x, min_y, min_z), (max_x, max_y, max_z) = bounds

        return (
            min_x <= x <= max_x
            and min_y <= y <= max_y
            and min_z <= z <= max_z
        )

    # -----------------------------
    # 2D convenience (grid-based)
    # -----------------------------
    def within_planning_bounds_2d(self, x, y):
        bounds = self.get_planning_bounds()
        if bounds is None:
            return False

        (min_x, min_y, _), (max_x, max_y, _) = bounds
        return min_x <= x <= max_x and min_y <= y <= max_y
