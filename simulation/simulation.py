import numpy as np

from .target_manager import TargetManagerMixin
from .exploration import ExplorationMixin
from .communication import CommunicationMixin
from .metrics import MetricsMixin


class Simulation(TargetManagerMixin, ExplorationMixin, CommunicationMixin, MetricsMixin):

    def __init__(self, world, belief_grid, dt=0.1):

        self.world = world
        self.belief_grid = belief_grid
        self.dt = dt
        self.drones = []
        self.time = 0.0

        # -----------------------------

        # Distributed exploration settings

        # -----------------------------

        # Distance at which an exploration goal is reached

        self.goal_tolerance = 2.0

        # Minimum distance between assigned frontier goals

        self.frontier_spacing = 8.0

        # -----------------------------

        # Target settings

        # -----------------------------

        # Distance at which a target is considered reached

        self.target_tolerance = 1.0

        # How long the whole swarm waits

        # when a target is found

        self.target_pause_duration = 3.0

        # Global swarm pause timer

        self.swarm_pause_timer = 0.0

        # -----------------------------

        # Performance timing

        # -----------------------------

        # Time when the first target is detected

        self.target_find_time = None

        # Time when map coverage reaches target percentage

        self.map_coverage_time = None

        # Coverage percentage required for completion

        self.coverage_target = 95.0

        # -----------------------------

        # High-level planning frequency

        # -----------------------------

        # Movement, sensing and avoidance still run every update.
        # Expensive frontier/connectivity planning runs less often.
        # At the GUI target of 60 FPS, 12 updates is about 5 planning cycles/second.

        self.planning_update_interval = 12
        self.planning_update_counter = 0

    def add_drone(self, drone):

        self.drones.append(drone)

    def sense_environment(self, drone):

        # Mark sensed free area as explored

        self.belief_grid.mark_explored(drone.position[0], drone.position[1], drone.sensing_radius)

        # -----------------------------

        # Detect obstacles

        # -----------------------------

        for obstacle in (self.world.obstacles):

            start_col = int(obstacle["x"] / self.belief_grid.cell_size)
            end_col = int((obstacle["x"] + obstacle["width"]) / self.belief_grid.cell_size)
            start_row = int(obstacle["y"] / self.belief_grid.cell_size)
            end_row = int((obstacle["y"] + obstacle["height"]) / self.belief_grid.cell_size)
            for row in range(start_row, end_row):

                for col in range(start_col, end_col):

                    cell_x = (col + 0.5) * self.belief_grid.cell_size
                    cell_y = (row + 0.5) * self.belief_grid.cell_size
                    dx = (cell_x - drone.position[0])
                    dy = (cell_y - drone.position[1])
                    distance = (dx ** 2 + dy ** 2) ** 0.5
                    if (distance <= drone.sensing_radius):

                        self.belief_grid.mark_obstacle(cell_x, cell_y)

        # -----------------------------

        # Detect targets

        # -----------------------------

        for target in (self.world.targets):

            dx = (target["x"] - drone.position[0])
            dy = (target["y"] - drone.position[1])
            distance = (dx ** 2 + dy ** 2) ** 0.5
            if (distance <= drone.sensing_radius):

                target["detected"] = (True)
                self.belief_grid.mark_target(target["x"], target["y"])

        # -----------------------------

        # Detect restricted zones

        # -----------------------------

        for zone in (self.world.restricted_zones):

            start_col = int(zone["x"] / self.belief_grid.cell_size)
            end_col = int((zone["x"] + zone["width"]) / self.belief_grid.cell_size)
            start_row = int(zone["y"] / self.belief_grid.cell_size)
            end_row = int((zone["y"] + zone["height"]) / self.belief_grid.cell_size)
            for row in range(start_row, end_row):

                for col in range(start_col, end_col):

                    cell_x = (col + 0.5) * self.belief_grid.cell_size
                    cell_y = (row + 0.5) * self.belief_grid.cell_size
                    dx = (cell_x - drone.position[0])
                    dy = (cell_y - drone.position[1])
                    distance = (dx ** 2 + dy ** 2) ** 0.5
                    if (distance <= drone.sensing_radius):

                        self.belief_grid.mark_restricted(cell_x, cell_y)

    def update(self):

        # -----------------------------

        # Whole swarm target pause

        # -----------------------------

        if (self.swarm_pause_timer > 0.0):

            self.update_target_pauses()
            self.time += (self.dt)
            return

        # -----------------------------

        # Sense using all drones first

        # -----------------------------

        for drone in (self.drones):

            self.sense_environment(drone)

        # -----------------------------

        # Record target / coverage times

        # -----------------------------

        self.update_performance_timers()

        # -----------------------------

        # Assign newly detected targets

        # -----------------------------

        self.assign_targets()

        # -----------------------------

        # Check target arrival

        # -----------------------------

        self.check_target_goals()

        # -----------------------------

        # Target may have been found

        # before movement

        # -----------------------------

        if (self.swarm_pause_timer > 0.0):

            self.update_target_pauses()
            self.time += (self.dt)
            return

        # -----------------------------

        # Communicate between drones

        # -----------------------------

        self.communicate_drones()

        # -----------------------------

        # Check reached exploration goals

        # -----------------------------

        # This is cheap and stays in the fast movement loop so a drone
        # immediately releases a goal when it reaches it.

        self.check_exploration_goals()

        # -----------------------------

        # High-level planning loop

        # -----------------------------

        # Frontier validation, connectivity repair and frontier assignment
        # are much more expensive than movement. They do not need to run
        # every rendered frame, so run them at a lower frequency.

        self.planning_update_counter += 1
        run_planning = (self.planning_update_counter >= self.planning_update_interval)

        # Run planning immediately if a non-target drone has no exploration
        # goal. This keeps startup and newly-freed drones responsive.

        needs_goal = any(
            drone.exploration_goal is None and drone.target_assignment is None
            for drone in self.drones
        )

        if (run_planning or needs_goal):

            self.planning_update_counter = 0
            self.validate_exploration_goals()
            self.maintain_swarm_connectivity()
            self.distribute_frontier_goals()

        # -----------------------------

        # Calculate movement decisions

        # -----------------------------

        new_velocities = []
        for drone in (self.drones):

            original_velocity = (drone.velocity.copy())

            # -----------------------------

            # Target goal has priority

            # -----------------------------

            if (drone.target_goal is not None):

                drone.move_toward_swarm_goal(drone.target_goal)

            # -----------------------------

            # Follow exploration goal

            # -----------------------------

            else:

                drone.move_toward_swarm_goal(drone.exploration_goal)

            # -----------------------------

            # Apply swarm behaviour

            # -----------------------------

            drone.apply_swarm_behaviour(self.drones)

            # -----------------------------

            # Separation safety

            # -----------------------------

            drone.enforce_separation(self.drones)

            # -----------------------------

            # Avoid known obstacles

            # -----------------------------

            drone.avoid_obstacles(self.belief_grid)

            # -----------------------------

            # Avoid known restricted zones

            # -----------------------------

            drone.avoid_restricted_zones(self.belief_grid)

            # -----------------------------

            # Store movement decision

            # -----------------------------

            new_velocities.append(drone.velocity.copy())
            drone.velocity = (original_velocity)

        # -----------------------------

        # Apply all movement together

        # -----------------------------

        for (drone, new_velocity) in zip(self.drones, new_velocities):

            drone.velocity = (new_velocity)
            drone.update(self.dt)
            self.world.handle_boundary(drone)

        # -----------------------------

        # Check targets after movement

        # -----------------------------

        self.check_target_goals()

        # -----------------------------

        # If target was found,

        # keep entire swarm stopped

        # -----------------------------

        if (self.swarm_pause_timer > 0.0):

            for drone in (self.drones):

                drone.velocity = np.array([0.0, 0.0], dtype=float)
                drone.acceleration = np.array([0.0, 0.0], dtype=float)

        # -----------------------------

        # Update simulation time

        # -----------------------------

        self.time += (self.dt)
