import numpy as np


class ExplorationMixin:

    def get_frontier_positions(self):

        frontier_cells = (self.belief_grid.get_frontiers())
        frontier_positions = []
        for (row, col) in frontier_cells:

            frontier_x = (col + 0.5) * self.belief_grid.cell_size
            frontier_y = (row + 0.5) * self.belief_grid.cell_size
            frontier_positions.append(np.array([frontier_x, frontier_y], dtype=float))

        return frontier_positions

    def check_exploration_goals(self):

        for drone in (self.drones):

            if (drone.exploration_goal is None):

                continue

            distance = (np.linalg.norm(drone.exploration_goal - drone.position))
            if (distance <= self.goal_tolerance):

                drone.exploration_goal = (None)

    def validate_exploration_goals(self):

        current_frontiers = set(self.belief_grid.get_frontiers())
        for drone in (self.drones):

            if (drone.exploration_goal is None):

                continue

            row, col = (self.belief_grid.world_to_grid(drone.exploration_goal[0], drone.exploration_goal[1]))
            if ((row, col) not in current_frontiers):

                drone.exploration_goal = (None)

    def get_communication_cost(self, drone, frontier):

        if (len(self.drones) <= 1):

            return 0.0

        nearest_distance = (float("inf"))
        for other_drone in (self.drones):

            if (other_drone.id == drone.id):

                continue

            distance = (np.linalg.norm(frontier - other_drone.position))
            if (distance < nearest_distance):

                nearest_distance = (distance)

        # -----------------------------

        # No communication penalty

        # -----------------------------

        if (nearest_distance <= drone.communication_radius):

            return 0.0

        excess_distance = (nearest_distance - drone.communication_radius)
        communication_penalty = (excess_distance * 3.0)
        return communication_penalty

    def get_predicted_connectivity_cost(self, drone, frontier):

        if (len(self.drones) <= 1):

            return 0.0

        # -----------------------------

        # Predict drone positions

        # -----------------------------

        predicted_positions = {}
        for other_drone in (self.drones):

            # Candidate drone is predicted

            # at frontier being tested

            if (other_drone.id == drone.id):

                predicted_positions[other_drone.id] = (frontier.copy())

            # -----------------------------

            # Target has priority

            # -----------------------------

            elif (other_drone.target_goal is not None):

                predicted_positions[other_drone.id] = (other_drone.target_goal.copy())

            # -----------------------------

            # Otherwise exploration goal

            # -----------------------------

            elif (other_drone.exploration_goal is not None):

                predicted_positions[other_drone.id] = (other_drone.exploration_goal.copy())

            # -----------------------------

            # Otherwise current position

            # -----------------------------

            else:

                predicted_positions[other_drone.id] = (other_drone.position.copy())

        # -----------------------------

        # Build predicted network

        # -----------------------------

        predicted_network = {}
        for first_drone in (self.drones):

            predicted_network[first_drone.id] = []
            for second_drone in (self.drones):

                if (first_drone.id == second_drone.id):

                    continue

                first_position = (predicted_positions[first_drone.id])
                second_position = (predicted_positions[second_drone.id])
                distance = (np.linalg.norm(first_position - second_position))
                communication_limit = min(first_drone.communication_radius, second_drone.communication_radius)
                if (distance <= communication_limit):

                    predicted_network[first_drone.id].append(second_drone.id)

        # -----------------------------

        # Find predicted components

        # -----------------------------

        visited = set()
        component_count = 0
        for current_drone in (self.drones):

            if (current_drone.id in visited):

                continue

            component_count += 1
            stack =[current_drone.id]
            while (len(stack) > 0):

                current_id = (stack.pop())
                if (current_id in visited):

                    continue

                visited.add(current_id)
                for neighbour_id in (predicted_network[current_id]):

                    if (neighbour_id not in visited):

                        stack.append(neighbour_id)

        if (component_count <= 1):

            return 0.0

        # -----------------------------

        # Connectivity penalty

        # -----------------------------

        connectivity_penalty = ((component_count - 1) * 40.0)
        return connectivity_penalty

    def _get_predicted_connectivity_costs(self, drone, frontiers):

        frontier_count = len(frontiers)
        if (frontier_count == 0 or len(self.drones) <= 1):

            return np.zeros(frontier_count, dtype=float)

        # Build predicted positions for every OTHER drone once.
        # Only the candidate drone changes position between frontiers.

        other_drones = [other for other in self.drones if (other.id != drone.id)]
        other_positions = []
        other_radii = []
        for other_drone in (other_drones):

            if (other_drone.target_goal is not None):

                predicted_position = (other_drone.target_goal)

            elif (other_drone.exploration_goal is not None):

                predicted_position = (other_drone.exploration_goal)

            else:

                predicted_position = (other_drone.position)

            other_positions.append(predicted_position)
            other_radii.append(other_drone.communication_radius)

        other_positions = np.asarray(other_positions, dtype=float)
        other_radii = np.asarray(other_radii, dtype=float)
        other_count = len(other_drones)

        # Build the fixed communication graph between the OTHER drones once.

        if (other_count == 1):

            component_labels = np.zeros(1, dtype=int)
            component_count = 1

        else:

            differences = other_positions[:, None, :] - other_positions[None, :, :]
            distances = np.linalg.norm(differences, axis=2)
            communication_limits = np.minimum(other_radii[:, None], other_radii[None, :])
            adjacency = (distances <= communication_limits)
            np.fill_diagonal(adjacency, False)

            component_labels = np.full(other_count, -1, dtype=int)
            component_count = 0
            for start_index in range(other_count):

                if (component_labels[start_index] != -1):

                    continue

                stack = [start_index]
                component_labels[start_index] = component_count
                while (len(stack) > 0):

                    current_index = stack.pop()
                    neighbours = np.flatnonzero(adjacency[current_index])
                    for neighbour_index in neighbours:

                        if (component_labels[neighbour_index] == -1):

                            component_labels[neighbour_index] = component_count
                            stack.append(int(neighbour_index))

                component_count += 1

        # Test ALL candidate frontiers against the fixed graph at once.
        # This gives the same component-count penalty as the previous method,
        # without rebuilding the complete network for every frontier.

        differences = frontiers[:, None, :] - other_positions[None, :, :]
        distances = np.linalg.norm(differences, axis=2)
        communication_limits = np.minimum(float(drone.communication_radius), other_radii)
        links = (distances <= communication_limits[None, :])

        connected_component_count = np.zeros(frontier_count, dtype=int)
        for component_id in range(component_count):

            component_mask = (component_labels == component_id)
            connected_component_count += np.any(links[:, component_mask], axis=1).astype(int)

        predicted_component_count = (component_count + 1 - connected_component_count)
        return np.maximum(predicted_component_count - 1, 0).astype(float) * 40.0

    def distribute_frontier_goals(self):

        # Do not change exploration goals while whole swarm is paused.

        if (self.swarm_pause_timer > 0.0):

            return

        frontier_positions = (self.get_frontier_positions())
        if (len(frontier_positions) == 0):

            return

        frontiers = np.asarray(frontier_positions, dtype=float)

        # Store active exploration goals.

        assigned_goals = []
        for drone in (self.drones):

            if (drone.target_assignment is not None):

                continue

            if (drone.exploration_goal is not None):

                assigned_goals.append(drone.exploration_goal.copy())

        # Assign drones without goals.

        for drone in (self.drones):

            if (drone.target_assignment is not None):

                continue

            if (drone.exploration_goal is not None):

                continue

            available_mask = np.ones(len(frontiers), dtype=bool)

            # Prevent duplicate exploration. This is the same spacing rule as
            # before, but all frontier-to-goal distances are calculated by NumPy.

            if (len(assigned_goals) > 0):

                assigned_array = np.asarray(assigned_goals, dtype=float)
                differences = frontiers[:, None, :] - assigned_array[None, :, :]
                assignment_distances = np.linalg.norm(differences, axis=2)
                available_mask = ~np.any(assignment_distances < self.frontier_spacing, axis=1)

            if (not np.any(available_mask)):

                continue

            available_frontiers = frontiers[available_mask]

            # Travel distance cost for every available frontier at once.

            distance_costs = np.linalg.norm(available_frontiers - drone.position, axis=1)

            # Communication cost for every available frontier at once.

            other_positions = np.asarray([
                other_drone.position for other_drone in self.drones
                if (other_drone.id != drone.id)
            ], dtype=float)

            if (len(other_positions) == 0):

                communication_costs = np.zeros(len(available_frontiers), dtype=float)

            else:

                differences = available_frontiers[:, None, :] - other_positions[None, :, :]
                distances = np.linalg.norm(differences, axis=2)
                nearest_distances = np.min(distances, axis=1)
                excess_distances = np.maximum(nearest_distances - drone.communication_radius, 0.0)
                communication_costs = excess_distances * 3.0

            # Exact predicted-connectivity penalty for every candidate at once.
            # The previous version rebuilt an N x N network separately for every
            # frontier. Here the fixed part of the graph is built only once.

            connectivity_costs = self._get_predicted_connectivity_costs(drone, available_frontiers)

            total_costs = distance_costs + communication_costs + connectivity_costs
            best_index = int(np.argmin(total_costs))
            best_frontier = available_frontiers[best_index]

            drone.exploration_goal = (best_frontier.copy())
            assigned_goals.append(best_frontier.copy())
