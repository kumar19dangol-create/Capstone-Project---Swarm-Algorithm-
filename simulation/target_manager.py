import numpy as np


class TargetManagerMixin:

    def is_target_assigned(self, target):

        for drone in (self.drones):

            if (drone.target_assignment is target):

                return True

        return False

    def assign_targets(self):

        # Do not create new assignments

        # while whole swarm is paused

        if (self.swarm_pause_timer > 0.0):

            return

        for target in (self.world.targets):

            # Target has not been discovered

            if not target["detected"]:

                continue

            # Target has already been completed

            if target["found"]:

                continue

            # Target already has a drone

            if self.is_target_assigned(target):

                continue

            available_drone = None
            nearest_distance = (float("inf"))
            target_position = np.array([target["x"], target["y"]], dtype=float)

            # -----------------------------

            # Find nearest available drone

            # -----------------------------

            for drone in (self.drones):

                if (drone.target_assignment is not None):

                    continue

                distance = (np.linalg.norm(target_position - drone.position))
                if (distance < nearest_distance):

                    nearest_distance = (distance)
                    available_drone = (drone)

            # -----------------------------

            # Assign nearest drone

            # -----------------------------

            if (available_drone is not None):

                available_drone.target_assignment = (target)
                available_drone.target_goal = (target_position.copy())

    def check_target_goals(self):

        # Already performing global pause

        if (self.swarm_pause_timer > 0.0):

            return

        for drone in (self.drones):

            if (drone.target_assignment is None):

                continue

            if (drone.target_goal is None):

                continue

            distance = (np.linalg.norm(drone.target_goal - drone.position))

            # -----------------------------

            # Target reached

            # -----------------------------

            if (distance <= self.target_tolerance):

                target = (drone.target_assignment)
                target["found"] = (True)
                drone.target_goal = (None)
                drone.target_assignment = (None)

                # -----------------------------

                # Start whole swarm pause

                # -----------------------------

                self.swarm_pause_timer = (self.target_pause_duration)

                # -----------------------------

                # Stop every drone

                # -----------------------------

                for swarm_drone in (self.drones):

                    swarm_drone.velocity = (np.array([0.0, 0.0], dtype=float))
                    swarm_drone.acceleration = (np.array([0.0, 0.0], dtype=float))

                break

    def update_target_pauses(self):

        if (self.swarm_pause_timer <= 0.0):

            return

        # -----------------------------

        # Keep whole swarm stopped

        # -----------------------------

        for drone in (self.drones):

            drone.velocity = (np.array([0.0, 0.0], dtype=float))
            drone.acceleration = (np.array([0.0, 0.0], dtype=float))

        # -----------------------------

        # Update timer

        # -----------------------------

        self.swarm_pause_timer -= (self.dt)

        # -----------------------------

        # Pause finished

        # -----------------------------

        if (self.swarm_pause_timer <= 0.0):

            self.swarm_pause_timer = (0.0)
