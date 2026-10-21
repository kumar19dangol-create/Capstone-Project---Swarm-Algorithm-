import numpy as np

class Drone:

    # -----------------------------
    # Initialise drone
    # -----------------------------

    def __init__(self, drone_id, x, y):
        self.id = drone_id
        self.position = np.array([x, y], dtype=float)
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.acceleration = np.array([0.0, 0.0], dtype=float)

        # -----------------------------
        # Sensor settings
        # -----------------------------
        # How far the drone can detect the environment
        self.sensing_radius = 5.0

        # -----------------------------
        # Obstacle avoidance settings
        # -----------------------------
        # Distance at which the drone starts avoiding a known obstacle
        self.avoidance_distance = 3.0

        # -----------------------------
        # Swarm settings
        # -----------------------------
        # Maximum distance for another drone to be considered a neighbour
        self.neighbour_radius = 8.0

        # Minimum preferred distance between drones
        self.separation_distance = 3.0

        # Boids behaviour weights
        self.separation_weight = 1.0
        self.alignment_weight = 0.3
        self.cohesion_weight = 0.2

        # Additional movement force weights
        self.goal_weight = 1.0
        self.obstacle_weight = 1.0
        self.restricted_weight = 1.0

        # -----------------------------
        # Movement settings
        # -----------------------------
        # Maximum allowed movement speed
        self.max_speed = 3.0

        # Normal movement speed
        self.desired_speed = 1.0

        # Controls how quickly the drone changes direction
        self.steering_rate = 0.08

        # Maximum combined correction from the Boids behaviours
        self.max_swarm_correction = 0.35

        # -----------------------------
        # Communication settings
        # -----------------------------
        # Maximum distance for direct drone-to-drone communication
        self.communication_radius = 15.0

        # -----------------------------
        # Communication memory
        # -----------------------------
        # Frontier currently assigned to this drone
        self.exploration_goal = None

        # Frontier assignments received from nearby drones
        self.known_assignments = {}

        # -----------------------------
        # Target settings
        # -----------------------------
        # Target currently assigned to this drone
        self.target_goal = None

        # Target data currently assigned to this drone
        self.target_assignment = None

        # Time remaining while stopped at a found target
        self.target_pause_timer = 0.0

    # -----------------------------
    # Update movement
    # -----------------------------

    def update(self, dt):
        self.velocity += self.acceleration * dt
        speed = np.linalg.norm(self.velocity)

        # Limit velocity to maximum speed
        if speed > self.max_speed:
            self.velocity = (self.velocity / speed) * self.max_speed

        # Update drone position
        self.position += self.velocity * dt

    # -----------------------------
    # Find nearest known obstacle
    # -----------------------------

    def get_nearest_obstacle(self, belief_grid, search_distance):
        nearest_cell = None
        nearest_distance = float("inf")

        # Only search cells close enough to affect this drone.
        # This replaces the old full-grid scan on every update.
        cell_size = belief_grid.cell_size
        min_col = max(0, int((self.position[0] - search_distance) / cell_size) - 1)
        max_col = min(belief_grid.cols - 1, int((self.position[0] + search_distance) / cell_size) + 1)
        min_row = max(0, int((self.position[1] - search_distance) / cell_size) - 1)
        max_row = min(belief_grid.rows - 1, int((self.position[1] + search_distance) / cell_size) + 1)

        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):

                if belief_grid.grid[row, col] != belief_grid.OBSTACLE:
                    continue

                cell_position = np.array([ (col + 0.5) * cell_size, (row + 0.5) * cell_size ], dtype=float)
                distance = np.linalg.norm(self.position - cell_position)

                if distance <= search_distance and distance < nearest_distance:
                    nearest_distance = distance
                    nearest_cell = cell_position

        return nearest_cell, nearest_distance

    # -----------------------------
    # Find nearest known restricted cell
    # -----------------------------

    def get_nearest_restricted(self, belief_grid, search_distance):
        nearest_cell = None
        nearest_distance = float("inf")

        # Only search cells close enough to affect this drone.
        cell_size = belief_grid.cell_size
        min_col = max(0, int((self.position[0] - search_distance) / cell_size) - 1)
        max_col = min(belief_grid.cols - 1, int((self.position[0] + search_distance) / cell_size) + 1)
        min_row = max(0, int((self.position[1] - search_distance) / cell_size) - 1)
        max_row = min(belief_grid.rows - 1, int((self.position[1] + search_distance) / cell_size) + 1)

        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):

                if belief_grid.grid[row, col] != belief_grid.RESTRICTED:
                    continue

                cell_position = np.array([ (col + 0.5) * cell_size, (row + 0.5) * cell_size ], dtype=float)
                distance = np.linalg.norm(self.position - cell_position)

                if distance <= search_distance and distance < nearest_distance:
                    nearest_distance = distance
                    nearest_cell = cell_position

        return nearest_cell, nearest_distance

    # -----------------------------
    # Reactive obstacle avoidance
    # -----------------------------

    def avoid_obstacles(self, belief_grid):

        if self.obstacle_weight <= 0.0:
            return

        # Higher weight reacts earlier; weight 1.0 preserves original distance
        effective_distance = self.avoidance_distance * max(1.0, self.obstacle_weight)
        obstacle_position, distance = self.get_nearest_obstacle(belief_grid, effective_distance)

        if obstacle_position is None:
            return

        # Calculate direction away from obstacle
        away_vector = self.position - obstacle_position
        away_distance = np.linalg.norm(away_vector)
        if away_distance < 0.001:
            return
        away_direction = away_vector / away_distance

        # Keep current movement speed
        current_speed = np.linalg.norm(self.velocity)
        if current_speed < 0.1:
            current_speed = self.desired_speed

        # Blend current direction with avoidance direction.
        # Weight 1.0 preserves the original direct-away behaviour.
        current_direction = self.velocity / current_speed
        turn_strength = min(1.0, self.obstacle_weight)
        desired_direction = current_direction * (1.0 - turn_strength) + away_direction * turn_strength
        desired_length = np.linalg.norm(desired_direction)
        if desired_length > 0.001:
            self.velocity = (desired_direction / desired_length) * current_speed

        # Reset acceleration
        self.acceleration = np.array([0.0, 0.0], dtype=float)

    # -----------------------------
    # Reactive restricted zone avoidance
    # -----------------------------

    def avoid_restricted_zones(self, belief_grid):

        if self.restricted_weight <= 0.0:
            return

        # Higher weight reacts earlier; weight 1.0 preserves original distance
        effective_distance = self.avoidance_distance * max(1.0, self.restricted_weight)
        restricted_position, distance = self.get_nearest_restricted(belief_grid, effective_distance)

        if restricted_position is None:
            return

        # Calculate direction away
        away_vector = self.position - restricted_position
        away_distance = np.linalg.norm(away_vector)
        if away_distance < 0.001:
            return
        away_direction = away_vector / away_distance

        # Keep current movement speed
        current_speed = np.linalg.norm(self.velocity)
        if current_speed < 0.1:
            current_speed = self.desired_speed

        # Blend current direction with restricted-zone avoidance direction.
        # Weight 1.0 preserves the original direct-away behaviour.
        current_direction = self.velocity / current_speed
        turn_strength = min(1.0, self.restricted_weight)
        desired_direction = current_direction * (1.0 - turn_strength) + away_direction * turn_strength
        desired_length = np.linalg.norm(desired_direction)
        if desired_length > 0.001:
            self.velocity = (desired_direction / desired_length) * current_speed

        # Reset acceleration
        self.acceleration = np.array([0.0, 0.0], dtype=float)

    # -----------------------------
    # Find nearby drones
    # -----------------------------

    def get_neighbours(self, drones):
        neighbours = []
        for other_drone in drones:

            # Do not count itself
            if other_drone.id == self.id:
                continue
            distance = np.linalg.norm(other_drone.position - self.position)

            # Add drones inside neighbour radius
            if distance <= self.neighbour_radius:
                neighbours.append(other_drone)
        return neighbours

    # -----------------------------
    # Get communication neighbours
    # -----------------------------

    def get_communication_neighbours(self, drones):
        communication_neighbours = []
        for other_drone in drones:

            # Do not communicate with itself
            if other_drone.id == self.id:
                continue

            # Calculate distance between drones
            difference = other_drone.position - self.position
            distance = np.linalg.norm(difference)

            # Drone is inside communication radius
            if distance <= self.communication_radius:
                communication_neighbours.append(other_drone)
        return communication_neighbours

    # -----------------------------
    # Share frontier assignments
    # -----------------------------

    def communicate(self, drones):
        communication_neighbours = self.get_communication_neighbours(drones)

        # Start with own assignment
        assignments = {}
        if self.exploration_goal is not None:
            assignments[self.id] = ( self.exploration_goal[0], self.exploration_goal[1] )

        # Receive assignments from nearby drones
        for neighbour in communication_neighbours:
            if neighbour.exploration_goal is not None:
                assignments[neighbour.id] = ( neighbour.exploration_goal[0], neighbour.exploration_goal[1] )

            # Receive information neighbour already knows
            for drone_id, goal in neighbour.known_assignments.items():
                assignments[drone_id] = goal
        self.known_assignments = assignments

    # -----------------------------
    # Separation
    # -----------------------------

    def calculate_separation(self, drones):
        separation = np.array([0.0, 0.0], dtype=float)
        neighbours = self.get_neighbours(drones)
        for neighbour in neighbours:
            difference = self.position - neighbour.position
            distance = np.linalg.norm(difference)

            # Separate when another drone is too close
            if distance < self.separation_distance and distance > 0.001:
                away_direction = difference / distance

                # Stronger when drones get closer
                strength = ( self.separation_distance - distance ) / self.separation_distance
                separation += away_direction * strength
        return separation

    # -----------------------------
    # Alignment
    # -----------------------------

    def calculate_alignment(self, drones):
        neighbours = self.get_neighbours(drones)

        # No neighbours means no alignment
        if len(neighbours) == 0:
            return np.array([0.0, 0.0], dtype=float)
        average_velocity = np.array([0.0, 0.0], dtype=float)
        for neighbour in neighbours:
            average_velocity += neighbour.velocity
        average_velocity /= len(neighbours)
        average_speed = np.linalg.norm(average_velocity)
        if average_speed < 0.001:
            return np.array([0.0, 0.0], dtype=float)
        alignment_direction = average_velocity / average_speed
        return alignment_direction

    # -----------------------------
    # Cohesion
    # -----------------------------

    def calculate_cohesion(self, drones):
        neighbours = self.get_neighbours(drones)

        # No neighbours means no cohesion
        if len(neighbours) == 0:
            return np.array([0.0, 0.0], dtype=float)
        centre = np.array([0.0, 0.0], dtype=float)
        for neighbour in neighbours:
            centre += neighbour.position
        centre /= len(neighbours)

        # Direction toward neighbour centre
        direction_to_centre = centre - self.position
        distance_to_centre = np.linalg.norm(direction_to_centre)
        if distance_to_centre < 0.001:
            return np.array([0.0, 0.0], dtype=float)
        cohesion_direction = direction_to_centre / distance_to_centre
        return cohesion_direction

    # -----------------------------
    # Apply swarm behaviour
    # -----------------------------

    def apply_swarm_behaviour(self, drones):

        # Get current movement direction
        current_speed = np.linalg.norm(self.velocity)
        if current_speed < 0.001:
            return
        forward_direction = self.velocity / current_speed

        # -----------------------------
        # Calculate Boids behaviours
        # -----------------------------
        separation = self.calculate_separation(drones)
        alignment = self.calculate_alignment(drones)
        cohesion = self.calculate_cohesion(drones)

        # -----------------------------
        # Prepare separation correction
        # -----------------------------
        separation_strength = np.linalg.norm(separation)
        if separation_strength > 0.001:
            separation_direction = separation / separation_strength

            # Check whether separation is parallel
            # or opposite to movement
            dot_value = np.dot( forward_direction, separation_direction )

            # Use sideways direction
            if abs(dot_value) > 0.8:
                perpendicular = np.array([ -forward_direction[1], forward_direction[0] ], dtype=float)

                # Alternate using drone ID
                if self.id % 2 == 0:
                    perpendicular *= -1
                separation_direction = perpendicular
        else:
            separation_direction = np.array([0.0, 0.0], dtype=float)

        # -----------------------------
        # Combine Boids behaviours
        # -----------------------------
        swarm_correction = ( separation_direction * self.separation_weight + alignment * self.alignment_weight + cohesion * self.cohesion_weight )

        # -----------------------------
        # Limit swarm correction
        # -----------------------------
        correction_length = np.linalg.norm(swarm_correction)
        if correction_length > self.max_swarm_correction:
            swarm_correction = ( swarm_correction / correction_length ) * self.max_swarm_correction

        # -----------------------------
        # Calculate desired direction
        # -----------------------------
        desired_direction = forward_direction + swarm_correction
        desired_length = np.linalg.norm(desired_direction)
        if desired_length < 0.001:
            return
        desired_direction = desired_direction / desired_length

        # -----------------------------
        # Smooth the turn
        # -----------------------------
        smooth_direction = ( forward_direction * (1.0 - self.steering_rate) + desired_direction * self.steering_rate )
        smooth_length = np.linalg.norm(smooth_direction)
        if smooth_length < 0.001:
            return
        smooth_direction = smooth_direction / smooth_length

        # -----------------------------
        # Maintain movement speed
        # -----------------------------
        self.velocity = smooth_direction * self.desired_speed

    # -----------------------------
    # Move toward assigned goal
    # -----------------------------

    def move_toward_swarm_goal(self, swarm_goal):

        # No active goal or goal attraction disabled
        if swarm_goal is None or self.goal_weight <= 0.0:
            return

        # Calculate direction toward goal
        goal_vector = swarm_goal - self.position
        goal_distance = np.linalg.norm(goal_vector)
        if goal_distance < 0.001:
            return
        goal_direction = goal_vector / goal_distance

        # Get current movement direction
        current_speed = np.linalg.norm(self.velocity)
        if current_speed < 0.001:
            current_direction = goal_direction
        else:
            current_direction = self.velocity / current_speed

        # Goal gently guides drone. Weight 1.0 preserves the original 10% pull.
        goal_strength = min(1.0, 0.1 * self.goal_weight)
        desired_direction = ( current_direction * (1.0 - goal_strength) + goal_direction * goal_strength )
        desired_length = np.linalg.norm(desired_direction)
        if desired_length < 0.001:
            return
        desired_direction = desired_direction / desired_length
        self.velocity = desired_direction * self.desired_speed

    # -----------------------------
    # Enforce separation safety
    # -----------------------------

    def enforce_separation(self, drones):
        strongest_away = np.array([0.0, 0.0], dtype=float)
        closest_distance = float("inf")
        for other_drone in drones:
            if other_drone.id == self.id:
                continue
            difference = self.position - other_drone.position
            distance = np.linalg.norm(difference)
            if ( distance < self.separation_distance and distance > 0.001 and distance < closest_distance ):
                closest_distance = distance
                strongest_away = difference / distance

        # No separation required
        if closest_distance == float("inf"):
            return

        # Stronger correction when extremely close
        danger_ratio = ( self.separation_distance - closest_distance ) / self.separation_distance
        current_speed = np.linalg.norm(self.velocity)
        if current_speed < 0.1:
            current_speed = self.desired_speed
        current_direction = self.velocity / current_speed
        desired_direction = ( current_direction + strongest_away * danger_ratio * 2.0 )
        desired_length = np.linalg.norm(desired_direction)
        if desired_length < 0.001:
            return
        desired_direction = desired_direction / desired_length
        self.velocity = desired_direction * self.desired_speed
