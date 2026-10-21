import math
import pygame

from models.belief import BeliefGrid
from .settings import *


class RendererMixin:

    def coverage_percent(self):

        return (self.simulation.get_coverage_percent())

    def draw_frame(self):

        self.screen.fill(BG)

        for rectangle in (self.header_rect, self.environment_rect, self.sidebar_rect, self.status_rect):

            pygame.draw.rect(self.screen, PANEL, rectangle)

            pygame.draw.rect(self.screen, INK, rectangle, 1)


        title = (self.title_font.render("AUTONOMOUS SWARM SIMULATOR", True, INK))

        self.screen.blit(title, title.get_rect(center=(self.header_rect.center)))

        self.draw_environment()

        self.draw_sidebar()

        self.draw_status()

        pygame.display.flip()

    def draw_environment(self):

        rectangle = (self.environment_rect)

        self.screen.set_clip(rectangle)

        cell_width = (rectangle.width / self.belief_grid.cols)

        cell_height = (rectangle.height / self.belief_grid.rows)


        # -----------------------------
        # Draw belief grid
        # -----------------------------

        for row in range(self.belief_grid.rows):

            for col in range(self.belief_grid.cols):

                state = (self.belief_grid.grid[row, col])

                left = (rectangle.x + col * cell_width)

                top = (rectangle.bottom - (row + 1) * cell_height)

                cell_rectangle = (
                    pygame.Rect(int(left), int(top), max(1, math.ceil(cell_width)), max(1, math.ceil(cell_height)))
                )

                if (state == BeliefGrid.EXPLORED):

                    pygame.draw.rect(self.screen, EXPLORED, cell_rectangle)

                elif (state == BeliefGrid.OBSTACLE):

                    pygame.draw.rect(self.screen, OBSTACLE, cell_rectangle)

                elif (state == BeliefGrid.TARGET):

                    pygame.draw.rect(self.screen, TARGET_DETECTED, cell_rectangle)

                elif (state == BeliefGrid.RESTRICTED):

                    pygame.draw.rect(self.screen, RESTRICTED, cell_rectangle)


        # -----------------------------
        # Draw grid lines
        # -----------------------------

        for col in range(self.belief_grid.cols + 1):

            x = int(rectangle.x + col * cell_width)

            pygame.draw.line(self.screen, VERY_LIGHT, (x, rectangle.y), (x, rectangle.bottom), 1)


        for row in range(self.belief_grid.rows + 1):

            y = int(rectangle.y + row * cell_height)

            pygame.draw.line(self.screen, VERY_LIGHT, (rectangle.x, y), (rectangle.right, y), 1)


        # -----------------------------
        # Draw real obstacles
        # -----------------------------

        for obstacle in (self.world.obstacles):

            left, bottom = (self.world_to_screen(obstacle["x"], obstacle["y"]))

            right, top = (
                self.world_to_screen(obstacle["x"] + obstacle["width"], obstacle["y"] + obstacle["height"])
            )

            obstacle_rectangle = (pygame.Rect(left, top, max(1, right - left), max(1, bottom - top)))

            pygame.draw.rect(self.screen, REAL_OBSTACLE, obstacle_rectangle)


        # -----------------------------
        # Draw restricted ground truth
        # -----------------------------

        for zone in (self.world.restricted_zones):

            left, bottom = (self.world_to_screen(zone["x"], zone["y"]))

            right, top = (self.world_to_screen(zone["x"] + zone["width"], zone["y"] + zone["height"]))

            zone_rectangle = (pygame.Rect(left, top, max(1, right - left), max(1, bottom - top)))

            pygame.draw.rect(self.screen, RESTRICTED, zone_rectangle, 1)


        # -----------------------------
        # Draw frontier cells
        # -----------------------------

        frontiers = (self.belief_grid.get_frontiers())

        for (row, col) in frontiers:

            left = (rectangle.x + col * cell_width)

            top = (rectangle.bottom - (row + 1) * cell_height)

            frontier_rectangle = (
                pygame.Rect(int(left), int(top), max(1, math.ceil(cell_width)), max(1, math.ceil(cell_height)))
            )

            pygame.draw.rect(self.screen, FRONTIER, frontier_rectangle, 2)


        # -----------------------------
        # Draw targets
        # -----------------------------

        for target in (self.world.targets):

            if not target["detected"]:

                continue

            target_x, target_y = (self.world_to_screen(target["x"], target["y"]))

            if target["found"]:

                target_colour = (TARGET_FOUND)

            else:

                target_colour = (TARGET_DETECTED)

            pygame.draw.circle(self.screen, target_colour, (target_x, target_y), 9)

            pygame.draw.circle(self.screen, BLACK, (target_x, target_y), 9, 2)


        # -----------------------------
        # Draw communication links
        # -----------------------------

        if self.developer_mode and self.visuals["communication_links"]:

            drawn_connections = (set())

            for drone in (self.simulation.drones):

                communication_neighbours = (drone.get_communication_neighbours(self.simulation.drones))

                drone_position = (self.world_to_screen(drone.position[0], drone.position[1]))

                for neighbour in (communication_neighbours):

                    connection = tuple(sorted((drone.id, neighbour.id)))

                    if (connection in drawn_connections):

                        continue

                    drawn_connections.add(connection)

                    neighbour_position = (self.world_to_screen(neighbour.position[0], neighbour.position[1]))

                    pygame.draw.line(self.screen, COMMUNICATION_LINK, drone_position, neighbour_position, 1)


        # -----------------------------
        # Draw neighbour links
        # -----------------------------

        if self.developer_mode and self.visuals["neighbour_links"]:

            drawn_neighbours = (set())

            for drone in (self.simulation.drones):

                neighbours = (drone.get_neighbours(self.simulation.drones))

                drone_position = (self.world_to_screen(drone.position[0], drone.position[1]))

                for neighbour in (neighbours):

                    connection = tuple(sorted((drone.id, neighbour.id)))

                    if (connection in drawn_neighbours):

                        continue

                    drawn_neighbours.add(connection)

                    neighbour_position = (self.world_to_screen(neighbour.position[0], neighbour.position[1]))

                    pygame.draw.line(self.screen, NEIGHBOUR_LINK, drone_position, neighbour_position, 2)


        # -----------------------------
        # Draw drones
        # -----------------------------

        scale_x = (rectangle.width / WORLD_WIDTH)

        scale_y = (rectangle.height / WORLD_HEIGHT)

        radius_scale = min(scale_x, scale_y)


        for drone in (self.simulation.drones):

            drone_x, drone_y = (self.world_to_screen(drone.position[0], drone.position[1]))


            # -----------------------------
            # Exploration goal
            # -----------------------------

            if ((not self.developer_mode or self.visuals["frontier_goals"]) and drone.exploration_goal is not None):

                goal_x, goal_y = (self.world_to_screen(drone.exploration_goal[0], drone.exploration_goal[1]))

                pygame.draw.circle(self.screen, EXPLORATION_GOAL, (goal_x, goal_y), 7)


            # -----------------------------
            # Target goal
            # -----------------------------

            if (drone.target_goal is not None):

                (target_goal_x, target_goal_y) = (self.world_to_screen(drone.target_goal[0], drone.target_goal[1]))

                pygame.draw.line(
                    self.screen,
                    TARGET_DETECTED,
                    (drone_x, drone_y),
                    (target_goal_x, target_goal_y),
                    2
                )


            # -----------------------------
            # Communication radius
            # -----------------------------

            if self.developer_mode and self.visuals["communication_radius"]:

                communication_pixels = int(drone.communication_radius * radius_scale)

                pygame.draw.circle(self.screen, COMMUNICATION_RADIUS, (drone_x, drone_y), communication_pixels, 1)


            # -----------------------------
            # Neighbour radius
            # -----------------------------

            if self.developer_mode and self.visuals["neighbour_radius"]:

                neighbour_pixels = int(drone.neighbour_radius * radius_scale)

                pygame.draw.circle(self.screen, NEIGHBOUR_RADIUS, (drone_x, drone_y), neighbour_pixels, 1)


            # -----------------------------
            # Sensing radius
            # -----------------------------

            if self.developer_mode and self.visuals["sensing"]:

                sensing_pixels = int(drone.sensing_radius * radius_scale)

                pygame.draw.circle(self.screen, SENSING_RADIUS, (drone_x, drone_y), sensing_pixels, 2)


            # -----------------------------
            # Whole swarm target pause ring
            # -----------------------------

            if (self.simulation.swarm_pause_timer > 0.0):

                pygame.draw.circle(self.screen, TARGET_FOUND, (drone_x, drone_y), 13, 3)


            # -----------------------------
            # Drone body
            # -----------------------------

            pygame.draw.circle(self.screen, DRONE, (drone_x, drone_y), 8)


            # -----------------------------
            # Direction indicator
            # -----------------------------

            velocity_x = float(drone.velocity[0])
            velocity_y = float(drone.velocity[1])

            velocity_length = math.hypot(velocity_x, velocity_y)

            if (velocity_length > 0.001):

                direction_x = (velocity_x / velocity_length)
                direction_y = (-velocity_y / velocity_length)

                side_x = (-direction_y)
                side_y = (direction_x)

                tip_distance = 15
                base_distance = 8
                half_width = 4

                tip = (
                    int(drone_x + direction_x * tip_distance),
                    int(drone_y + direction_y * tip_distance)
                )

                left = (
                    int(drone_x + direction_x * base_distance + side_x * half_width),
                    int(drone_y + direction_y * base_distance + side_y * half_width)
                )

                right = (
                    int(drone_x + direction_x * base_distance - side_x * half_width),
                    int(drone_y + direction_y * base_distance - side_y * half_width)
                )

                pygame.draw.polygon(self.screen, DRONE, [tip, left, right])

                pygame.draw.polygon(self.screen, BLACK, [tip, left, right], 1)


            # -----------------------------
            # Drone ID
            # -----------------------------

            id_text = (self.small_font.render(str(drone.id), True, INK))

            self.screen.blit(id_text, (drone_x + 10, drone_y - 16))


        self.screen.set_clip(None)


        # -----------------------------
        # Environment heading
        # -----------------------------

        heading = (self.heading_font.render("2D ENVIRONMENT", True, INK))

        self.screen.blit(heading, (rectangle.x + 18, rectangle.y + 14))


        # -----------------------------
        # Placement hint
        # -----------------------------

        hint_text = None

        if (self.placement_mode in PAINT_MODES):

            hint_text = (
                f"Painting "
                f"{self.placement_mode.upper()} "
                f"| left = paint "
                f"| right = erase "
                f"| brush = "
                f"{self.brush_radius:g}"
            )

            mouse_position = (pygame.mouse.get_pos())

            if (rectangle.collidepoint(mouse_position)):

                brush_colour = (OBSTACLE_EDGE if self.placement_mode == "obstacle" else RESTRICTED_EDGE)

                brush_pixels = max(3, int(self.brush_radius * radius_scale))

                self.screen.set_clip(rectangle)

                pygame.draw.circle(self.screen, brush_colour, mouse_position, brush_pixels, 2)

                self.screen.set_clip(None)


        elif (self.placement_mode == "target"):

            hint_text = ("Click in environment " "to place TARGET")


        if (hint_text is not None):

            hint = (self.small_font.render(hint_text, True, MID))

            self.screen.blit(hint, (rectangle.x + 18, rectangle.bottom - 28))

    def draw_sidebar(self):

        x = (self.sidebar_rect.x + 18)

        for (title, y) in self.sidebar_headings:

            heading = (self.heading_font.render(title, True, INK))

            self.screen.blit(heading, (x, y))


        for row in self.sidebar_rows:

            if len(row) == 4:
                label, key, y, label_x = row
            else:
                label, key, y = row
                label_x = x

            label_surface = (self.font.render(label, True, INK))

            self.screen.blit(label_surface, (label_x, y))

            self.fields[key].draw(self.screen, self.font)


        self.buttons["mode"].draw(self.screen, self.small_font, active=self.developer_mode)


        for key in ("obstacle", "target", "restricted"):

            self.buttons[key].draw(self.screen, self.small_font, active=(self.placement_mode == key))


        self.buttons["save_map"].draw(self.screen, self.small_font, active=False)

        self.buttons["load_map"].draw(self.screen, self.small_font, active=False)


        if self.developer_mode:

            visual_buttons = {
                "toggle_sensing": "sensing",
                "toggle_neighbour_radius": "neighbour_radius",
                "toggle_communication_radius": "communication_radius",
                "toggle_communication_links": "communication_links",
                "toggle_neighbour_links": "neighbour_links",
                "toggle_frontier_goals": "frontier_goals"
            }

            for button_key, visual_key in visual_buttons.items():

                self.buttons[button_key].draw(self.screen, self.small_font, active=self.visuals[visual_key])


        self.buttons["run"].draw(self.screen, self.font, active=(self.running_simulation))

        self.buttons["pause"].draw(self.screen, self.font, active=(not self.running_simulation))

        self.buttons["reset"].draw(self.screen, self.font, active=False)

    def draw_status(self):

        components = (self.simulation.get_communication_components())

        if (len(self.simulation.drones) == 0):

            connected_count = 0

        elif (len(components) == 0):

            connected_count = 0

        else:

            connected_count = max(len(component) for component in components)


        total_drones = len(self.simulation.drones)

        detected_targets = sum(1 for target in self.world.targets if target["detected"])

        found_targets = sum(1 for target in self.world.targets if target["found"])


        # -----------------------------
        # Target find time
        # -----------------------------

        if (len(self.world.targets) == 0):

            target_time_text = ("No target")

        elif (self.simulation.target_find_time is None):

            target_time_text = ("Searching...")

        else:

            target_time_text = (f"{self.simulation.target_find_time:.1f}s")


        # -----------------------------
        # Map coverage time
        # -----------------------------

        if (self.simulation.map_coverage_time is None):

            coverage_time_text = ("Exploring...")

        else:

            coverage_time_text = (f"{self.simulation.map_coverage_time:.1f}s")


        # -----------------------------
        # Main status
        # -----------------------------

        status = (

            f"Coverage: "
            f"{self.coverage_percent():5.1f}%    "

            f"Connected: "
            f"{connected_count}/"
            f"{total_drones}    "

            f"Targets: "
            f"{found_targets}/"
            f"{len(self.world.targets)} "
            f"found    "

            f"Detected: "
            f"{detected_targets}"
        )


        text = (self.small_font.render(status, True, INK))

        self.screen.blit(text, (self.status_rect.x + 16, self.status_rect.y + 7))


        # -----------------------------
        # Performance times
        # -----------------------------

        timing_status = (f"Target Find: " f"{target_time_text}    " f"95% Coverage: " f"{coverage_time_text}")


        # -----------------------------
        # Swarm pause status
        # -----------------------------

        if (self.simulation.swarm_pause_timer > 0.0):

            timing_status += (f"    SWARM PAUSED: " f"{self.simulation.swarm_pause_timer:.1f}s")


        timing_text = (self.small_font.render(timing_status, True, INK))

        self.screen.blit(timing_text, (self.status_rect.x + 16, self.status_rect.y + 29))


        # -----------------------------
        # Status message
        # -----------------------------

        message = (self.small_font.render(self.status_message, True, MID))

        self.screen.blit(message, (self.status_rect.right - message.get_width() - 16, self.status_rect.y + 7))
