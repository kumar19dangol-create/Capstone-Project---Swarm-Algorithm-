class World:

    # -----------------------------
    # Initialise world
    # -----------------------------

    def __init__(self, width, height):

        self.width = width
        self.height = height

        # Obstacles that physically exist in the environment
        self.obstacles = []

        # Targets that actually exist in the world
        self.targets = []

        # Restricted areas that exist in the world
        self.restricted_zones = []


    # -----------------------------
    # Add obstacle
    # -----------------------------

    def add_obstacle(self, x, y, width, height):

        obstacle = {"x": x, "y": y, "width": width, "height": height}

        self.obstacles.append(obstacle)


    # -----------------------------
    # Add target
    # -----------------------------

    def add_target(self, x, y):

        target = {"x": x, "y": y, "detected": False, "found": False}

        self.targets.append(target)


    # -----------------------------
    # Add restricted zone
    # -----------------------------

    def add_restricted_zone(self, x, y, width, height):

        zone = {"x": x, "y": y, "width": width, "height": height}

        self.restricted_zones.append(zone)


    # -----------------------------
    # Check world boundaries
    # -----------------------------

    def is_inside(self, x, y):

        return 0 <= x <= self.width and 0 <= y <= self.height


    # -----------------------------
    # Handle world boundary
    # -----------------------------

    def handle_boundary(self, drone):

        # Left boundary
        if drone.position[0] < 0:

            drone.position[0] = 0
            drone.velocity[0] *= -1

        # Right boundary
        elif drone.position[0] > self.width:

            drone.position[0] = self.width
            drone.velocity[0] *= -1

        # Bottom boundary
        if drone.position[1] < 0:

            drone.position[1] = 0
            drone.velocity[1] *= -1

        # Top boundary
        elif drone.position[1] > self.height:

            drone.position[1] = self.height
            drone.velocity[1] *= -1