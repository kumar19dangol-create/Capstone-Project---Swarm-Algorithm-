import numpy as np


class BeliefGrid:

    # -----------------------------
    # Cell states
    # -----------------------------

    UNKNOWN = 0
    EXPLORED = 1
    OBSTACLE = 2
    TARGET = 3
    RESTRICTED = 4


    # -----------------------------
    # Initialise belief grid
    # -----------------------------

    def __init__(self, world_width, world_height, cell_size=1.0):

        self.cell_size = cell_size

        self.cols = int(world_width / cell_size)
        self.rows = int(world_height / cell_size)

        self.grid = np.full((self.rows, self.cols), self.UNKNOWN, dtype=np.uint8)

        # Cached frontier cells. Frontier detection is requested by both the
        # planner and renderer, so avoid repeating the same full-map work
        # when the belief grid has not changed.
        self._frontiers_cache = []
        self._frontiers_dirty = True


    # -----------------------------
    # Convert world to grid
    # -----------------------------

    def world_to_grid(self, x, y):

        col = int(x / self.cell_size)
        row = int(y / self.cell_size)

        col = min(max(col, 0), self.cols - 1)
        row = min(max(row, 0), self.rows - 1)

        return row, col


    # -----------------------------
    # Mark sensed cells as explored
    # -----------------------------

    def mark_explored(self, x, y, sensing_radius):

        center_row, center_col = self.world_to_grid(x, y)
        cell_radius = int(sensing_radius / self.cell_size)

        for row in range(center_row - cell_radius, center_row + cell_radius + 1):

            for col in range(center_col - cell_radius, center_col + cell_radius + 1):

                if 0 <= row < self.rows and 0 <= col < self.cols:

                    cell_x = (col + 0.5) * self.cell_size
                    cell_y = (row + 0.5) * self.cell_size

                    dx = cell_x - x
                    dy = cell_y - y

                    distance = np.sqrt(dx ** 2 + dy ** 2)

                    if distance <= sensing_radius:

                        if self.grid[row, col] == self.UNKNOWN:
                            self.grid[row, col] = self.EXPLORED
                            self._frontiers_dirty = True


    # -----------------------------
    # Mark detected obstacle
    # -----------------------------

    def mark_obstacle(self, x, y):

        row, col = self.world_to_grid(x, y)

        if self.grid[row, col] != self.OBSTACLE:
            self.grid[row, col] = self.OBSTACLE
            self._frontiers_dirty = True


    # -----------------------------
    # Mark detected target
    # -----------------------------

    def mark_target(self, x, y):

        row, col = self.world_to_grid(x, y)

        if self.grid[row, col] != self.TARGET:
            self.grid[row, col] = self.TARGET
            self._frontiers_dirty = True


    # -----------------------------
    # Mark restricted cell
    # -----------------------------

    def mark_restricted(self, x, y):

        row, col = self.world_to_grid(x, y)

        if self.grid[row, col] != self.RESTRICTED:
            self.grid[row, col] = self.RESTRICTED
            self._frontiers_dirty = True


    # -----------------------------
    # Find frontier cells
    # -----------------------------

    def get_frontiers(self):

        # Return the cached result if no belief cell changed since the last
        # frontier calculation. This lets planning and rendering reuse it.
        if not self._frontiers_dirty:
            return self._frontiers_cache

        # Vectorised frontier detection. A frontier is an EXPLORED cell with
        # at least one in-bounds UNKNOWN 4-neighbour. NumPy performs these
        # whole-grid comparisons in compiled code instead of nested Python
        # loops over every row and column.
        explored = (self.grid == self.EXPLORED)
        unknown = (self.grid == self.UNKNOWN)
        adjacent_unknown = np.zeros_like(unknown, dtype=bool)

        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]

        frontier_mask = explored & adjacent_unknown
        rows, cols = np.nonzero(frontier_mask)

        self._frontiers_cache = list(zip(rows.tolist(), cols.tolist()))
        self._frontiers_dirty = False
        return self._frontiers_cache


    # -----------------------------
    # Find nearest safe frontier
    # -----------------------------

    def get_nearest_frontier(self, x, y):

        frontiers = self.get_frontiers()

        if len(frontiers) == 0:
            return None

        nearest_frontier = None
        nearest_distance = float("inf")

        for row, col in frontiers:

            # Do not use obstacle or restricted cells
            if (
                self.grid[row, col] == self.OBSTACLE
                or self.grid[row, col] == self.RESTRICTED
            ):
                continue

            frontier_x = (col + 0.5) * self.cell_size
            frontier_y = (row + 0.5) * self.cell_size

            dx = frontier_x - x
            dy = frontier_y - y

            distance = (dx ** 2 + dy ** 2) ** 0.5

            if distance < nearest_distance:

                nearest_distance = distance

                nearest_frontier = (
                    frontier_x,
                    frontier_y
                )

        return nearest_frontier