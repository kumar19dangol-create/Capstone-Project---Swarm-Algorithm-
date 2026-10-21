class MetricsMixin:

    def get_coverage_percent(self):

        known = (self.belief_grid.grid != self.belief_grid.UNKNOWN).sum()
        total = (self.belief_grid.grid.size)
        if total == 0:

            return 0.0

        return (100.0 * known / total)

    def update_performance_timers(self):

        # -----------------------------

        # Target find time

        # -----------------------------

        if (self.target_find_time is None):

            target_detected = any(target["detected"] for target in self.world.targets)
            if target_detected:

                self.target_find_time = (self.time)

        # -----------------------------

        # Map coverage time

        # -----------------------------

        if (self.map_coverage_time is None):

            coverage = (self.get_coverage_percent())
            if (coverage >= self.coverage_target):

                self.map_coverage_time = (self.time)
