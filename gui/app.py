import ctypes
import json
import math
import random
from pathlib import Path

try:
    ctypes.windll.shcore.SetProcessDpiAwareness(2)
except (AttributeError, OSError):
    pass

import pygame

from models.belief import BeliefGrid
from models.drone import Drone
from models.world import World
from simulation.simulation import Simulation
from .components import NumericField, Button
from .map_editor import MapEditorMixin
from .renderer import RendererMixin
from .settings import *


class SwarmGUI(MapEditorMixin, RendererMixin):

    def __init__(self):

        pygame.init()

        # -----------------------------
        # Create full-screen window
        # -----------------------------

        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        self.window_width, self.window_height = self.screen.get_size()

        pygame.display.set_caption("Autonomous Swarm Simulator")

        self.clock = (pygame.time.Clock())


        # -----------------------------
        # Fonts
        # -----------------------------

        self.font = (pygame.font.SysFont("consolas", 17))

        self.small_font = (pygame.font.SysFont("consolas", 14))

        self.heading_font = (pygame.font.SysFont("consolas", 19, bold=True))

        self.title_font = (pygame.font.SysFont("consolas", 22, bold=True))


        # -----------------------------
        # Main GUI areas
        # -----------------------------

        self.header_rect = (pygame.Rect(OUTER_MARGIN, OUTER_MARGIN, self.window_width - 2 * OUTER_MARGIN, HEADER_HEIGHT))

        content_top = (self.header_rect.bottom)

        content_bottom = (self.window_height - OUTER_MARGIN - STATUS_HEIGHT)

        sidebar_x = (self.window_width - OUTER_MARGIN - SIDEBAR_WIDTH)

        available_width = (sidebar_x - GAP - OUTER_MARGIN)

        available_height = (content_bottom - content_top)

        self.environment_rect = (pygame.Rect(OUTER_MARGIN, content_top, available_width, available_height))

        self.sidebar_rect = (pygame.Rect(sidebar_x, content_top, SIDEBAR_WIDTH, available_height))

        self.status_rect = (pygame.Rect(OUTER_MARGIN, content_bottom, self.window_width - 2 * OUTER_MARGIN, STATUS_HEIGHT))


        # -----------------------------
        # Load configuration
        # -----------------------------

        config_path = Path(__file__).resolve().parent.parent / "config.json"

        with config_path.open("r", encoding="utf-8") as config_file:

            self.config = json.load(config_file)

        swarm_config = self.config["swarm"]
        force_config = self.config["forces"]
        limits_config = self.config["limits"]
        editor_config = self.config["editor"]
        rendering_config = self.config["rendering"]


        # -----------------------------
        # Numeric controls
        # -----------------------------

        self.fields = {

            "drones": NumericField("drones", swarm_config["drone_count"], int, limits_config["drone_count_min"], limits_config["drone_count_max"]),

            "sense": NumericField("sense", swarm_config["sensing_radius"], float, limits_config["sensing_radius_min"], limits_config["sensing_radius_max"]),

            "neighbour": NumericField("neighbour", swarm_config["neighbour_radius"], float, limits_config["neighbour_radius_min"], limits_config["neighbour_radius_max"]),

            "comm": NumericField("comm", swarm_config["communication_radius"], float, limits_config["communication_radius_min"], limits_config["communication_radius_max"]),

            "separation": NumericField("separation", force_config["separation_weight"], float, limits_config["force_weight_min"], limits_config["force_weight_max"]),

            "alignment": NumericField("alignment", force_config["alignment_weight"], float, limits_config["force_weight_min"], limits_config["force_weight_max"]),

            "cohesion": NumericField("cohesion", force_config["cohesion_weight"], float, limits_config["force_weight_min"], limits_config["force_weight_max"]),

            "goal": NumericField("goal", force_config["goal_weight"], float, limits_config["force_weight_min"], limits_config["force_weight_max"]),

            "obstacle_force": NumericField("obstacle_force", force_config["obstacle_weight"], float, limits_config["force_weight_min"], limits_config["force_weight_max"]),

            "restricted_force": NumericField("restricted_force", force_config["restricted_weight"], float, limits_config["force_weight_min"], limits_config["force_weight_max"]),

            "brush": NumericField("brush", editor_config["brush_default"], float, editor_config["brush_min"], editor_config["brush_max"])
        }


        # -----------------------------
        # Buttons
        # -----------------------------

        self.buttons = {

            "obstacle": Button("PAINT OBSTACLE", "obstacle", OBSTACLE_TINT, OBSTACLE_EDGE),

            "target": Button("ADD TARGET", "target", TARGET_TINT, TARGET_EDGE),

            "restricted": Button("PAINT RESTRICTED", "restricted", RESTRICTED_TINT, RESTRICTED_EDGE),

            "mode": Button("MODE: NORMAL", "mode"),

            "save_map": Button("SAVE MAP", "save_map"),

            "load_map": Button("LOAD MAP", "load_map"),

            "toggle_sensing": Button("SENSE CIRCLE", "toggle_sensing"),

            "toggle_neighbour_radius": Button("NEIGHBOUR CIRCLE", "toggle_neighbour_radius"),

            "toggle_communication_radius": Button("COMM CIRCLE", "toggle_communication_radius"),

            "toggle_communication_links": Button("COMM LINKS", "toggle_communication_links"),

            "toggle_neighbour_links": Button("NEIGHBOUR LINKS", "toggle_neighbour_links"),

            "toggle_frontier_goals": Button("FRONTIER GOALS", "toggle_frontier_goals"),

            "run": Button("RUN", "run"),

            "pause": Button("PAUSE", "pause"),

            "reset": Button("RESET", "reset")
        }


        # -----------------------------
        # GUI state
        # -----------------------------

        self.running_simulation = False

        self.placement_mode = None

        self.brush_radius = float(editor_config["brush_default"])

        self.painting = None

        self.last_paint_pos = None

        self.obstacle_cells = {}

        self.restricted_cells = set()

        self.status_message = ("Ready")

        self.developer_mode = False

        self.visible_button_keys = set()


        # -----------------------------
        # Visual settings
        # -----------------------------

        self.visuals = {
            "sensing": bool(rendering_config["show_sensing_radius"]),
            "neighbour_radius": bool(rendering_config["show_neighbour_radius"]),
            "communication_radius": bool(rendering_config["show_communication_radius"]),
            "communication_links": bool(rendering_config["show_communication_links"]),
            "neighbour_links": bool(rendering_config["show_neighbour_links"]),
            "frontier_goals": bool(rendering_config["show_frontier_goals"])
        }


        # -----------------------------
        # Create simulation
        # -----------------------------

        self.build_simulation()

        self.layout_controls()

    def build_simulation(self):

        self.world = World(WORLD_WIDTH, WORLD_HEIGHT)

        self.belief_grid = BeliefGrid(world_width=self.world.width, world_height=self.world.height, cell_size=1.0)

        self.simulation = Simulation(self.world, self.belief_grid, dt=DT)

    def is_safe_spawn_position(self, x, y, existing_positions):

        spawn_clearance = 2.0
        drone_spacing = 4.0


        # -----------------------------
        # Keep away from world edges
        # -----------------------------

        if (
            x < spawn_clearance
            or x > WORLD_WIDTH - spawn_clearance
            or y < spawn_clearance
            or y > WORLD_HEIGHT - spawn_clearance
        ):

            return False


        # -----------------------------
        # Keep away from obstacles
        # -----------------------------

        for obstacle in (self.world.obstacles):

            if (
                obstacle["x"] - spawn_clearance
                <= x
                <= obstacle["x"]
                + obstacle["width"]
                + spawn_clearance
                and
                obstacle["y"] - spawn_clearance
                <= y
                <= obstacle["y"]
                + obstacle["height"]
                + spawn_clearance
            ):

                return False


        # -----------------------------
        # Keep away from restricted zones
        # -----------------------------

        for zone in (self.world.restricted_zones):

            if (
                zone["x"] - spawn_clearance
                <= x
                <= zone["x"]
                + zone["width"]
                + spawn_clearance
                and
                zone["y"] - spawn_clearance
                <= y
                <= zone["y"]
                + zone["height"]
                + spawn_clearance
            ):

                return False


        # -----------------------------
        # Keep away from targets
        # -----------------------------

        for target in (self.world.targets):

            distance = math.hypot(x - target["x"], y - target["y"])

            if (distance < spawn_clearance):

                return False


        # -----------------------------
        # Keep drones separated at spawn
        # -----------------------------

        for (other_x, other_y) in existing_positions:

            distance = math.hypot(x - other_x, y - other_y)

            if (distance < drone_spacing):

                return False


        return True

    def spawn_drones(self):

        if (len(self.simulation.drones) > 0):

            return

        count = int(self.fields["drones"].value)

        spawn_positions = []

        for index in range(count):

            spawn_position = None


            # -----------------------------
            # Find random safe position
            # -----------------------------

            for attempt in range(2000):

                x = random.uniform(2.0, WORLD_WIDTH - 2.0)

                y = random.uniform(2.0, WORLD_HEIGHT - 2.0)

                if self.is_safe_spawn_position(x, y, spawn_positions):

                    spawn_position = (x, y)

                    break


            # -----------------------------
            # Stop if no safe space exists
            # -----------------------------

            if (spawn_position is None):

                self.status_message = ("Not enough safe space " "for all drones")

                break

            x, y = spawn_position

            spawn_positions.append(spawn_position)

            drone = Drone(index + 1, x, y)


            # -----------------------------
            # Apply GUI settings
            # -----------------------------

            drone.sensing_radius = float(self.fields["sense"].value)

            drone.neighbour_radius = float(self.fields["neighbour"].value)

            drone.communication_radius = float(self.fields["comm"].value)

            drone.separation_weight = float(self.fields["separation"].value)

            drone.alignment_weight = float(self.fields["alignment"].value)

            drone.cohesion_weight = float(self.fields["cohesion"].value)

            drone.goal_weight = float(self.fields["goal"].value)

            drone.obstacle_weight = float(self.fields["obstacle_force"].value)

            drone.restricted_weight = float(self.fields["restricted_force"].value)


            # -----------------------------
            # Initial velocity
            # -----------------------------

            drone.velocity[0] = float(self.config["swarm"]["initial_velocity_x"])

            drone.velocity[1] = float(self.config["swarm"]["initial_velocity_y"])


            # -----------------------------
            # Add drone
            # -----------------------------

            self.simulation.add_drone(drone)

    def apply_settings(self, key):

        if key == "drones":

            self.status_message = ("Drone count set for next run")

            return


        if key == "sense":

            for drone in (self.simulation.drones):

                drone.sensing_radius = float(self.fields["sense"].value)


        elif key == "neighbour":

            for drone in (self.simulation.drones):

                drone.neighbour_radius = float(self.fields["neighbour"].value)


        elif key == "comm":

            for drone in (self.simulation.drones):

                drone.communication_radius = float(self.fields["comm"].value)


        elif key == "separation":

            for drone in (self.simulation.drones):

                drone.separation_weight = float(self.fields["separation"].value)


        elif key == "alignment":

            for drone in (self.simulation.drones):

                drone.alignment_weight = float(self.fields["alignment"].value)


        elif key == "cohesion":

            for drone in (self.simulation.drones):

                drone.cohesion_weight = float(self.fields["cohesion"].value)


        elif key == "goal":

            for drone in (self.simulation.drones):

                drone.goal_weight = float(self.fields["goal"].value)


        elif key == "obstacle_force":

            for drone in (self.simulation.drones):

                drone.obstacle_weight = float(self.fields["obstacle_force"].value)


        elif key == "restricted_force":

            for drone in (self.simulation.drones):

                drone.restricted_weight = float(self.fields["restricted_force"].value)


        elif key == "brush":

            self.brush_radius = float(self.fields["brush"].value)

            self.status_message = (f"Brush size: {self.brush_radius:g}")

            return


        self.status_message = ("Settings updated")
        

    def rebuild_swarm(self):

        obstacles = [obstacle.copy() for obstacle in self.world.obstacles]

        targets = [
            {"x": target["x"], "y": target["y"], "detected": False, "found": False}
            for target
            in self.world.targets
        ]

        restricted_zones = [zone.copy() for zone in self.world.restricted_zones]

        self.build_simulation()

        self.world.obstacles = (obstacles)

        self.world.targets = (targets)

        self.world.restricted_zones = (restricted_zones)

        self.running_simulation = (False)

    def reset(self):

        self.obstacle_cells = {}

        self.restricted_cells = (set())

        self.build_simulation()

        self.running_simulation = (False)

        self.placement_mode = None

        self.painting = None

        self.last_paint_pos = None

        self.status_message = ("Environment cleared")

    def layout_controls(self):

        x = (self.sidebar_rect.x + 18)

        right = (self.sidebar_rect.right - 18)

        field_x = (right - 72)

        button_width = (right - x)

        row_height = 40

        y = (self.sidebar_rect.y + 18)

        self.sidebar_headings = []

        self.sidebar_rows = []

        self.visible_button_keys = {
            "mode", "obstacle", "target", "restricted",
            "save_map", "load_map", "run", "pause", "reset"
        }


        def section(title, rows):

            nonlocal y

            self.sidebar_headings.append((title, y))

            y += 44

            for label, key in rows:

                self.sidebar_rows.append((label, key, y))

                self.fields[key].set_rect((field_x, y - 5, 66, 28))

                y += row_height

            y += 10


        # -----------------------------
        # Mode switch
        # -----------------------------

        self.buttons["mode"].label = ("MODE: DEVELOPER" if self.developer_mode else "MODE: NORMAL")

        self.buttons["mode"].set_rect((x, y, button_width, 34))

        y += 50


        # -----------------------------
        # Normal controls
        # -----------------------------

        section("SIMULATION", [("Drones:", "drones")])

        section("ENVIRONMENT", [("Brush size:", "brush")])

        for key in ("obstacle", "target", "restricted"):

            self.buttons[key].set_rect((x, y, button_width, 34))

            y += row_height


        # -----------------------------
        # Developer controls
        # -----------------------------

        if self.developer_mode:

            section("DEVELOPER", [("Sense radius:", "sense"),  ("Neighbour radius:", "neighbour"), ("Comm radius:", "comm")])

            self.sidebar_headings.append(("SIX MOVEMENT FORCES", y))
            y += 42

            force_rows = [
                ("Separation:", "separation"),
                ("Alignment:", "alignment"),
                ("Cohesion:", "cohesion"),
                ("Goal attraction:", "goal"),
                ("Obstacle avoidance:", "obstacle_force"),
                ("Restricted avoidance:", "restricted_force")
            ]

            for label, key in force_rows:

                self.sidebar_rows.append((label, key, y))

                self.fields[key].set_rect((field_x, y - 5, 66, 28))

                y += 36

            y += 8

            self.sidebar_headings.append(("VISUALS", y))

            y += 38

            half = (button_width - 8) // 2

            visual_rows = [
                ("toggle_sensing", "toggle_neighbour_radius"),
                ("toggle_communication_radius", "toggle_communication_links"),
                ("toggle_neighbour_links", "toggle_frontier_goals")
            ]

            for left_key, right_key in visual_rows:

                self.buttons[left_key].set_rect((x, y, half, 30))

                self.buttons[right_key].set_rect((x + half + 8, y, half, 30))

                self.visible_button_keys.add(left_key)

                self.visible_button_keys.add(right_key)

                y += 36


        half = (button_width - 8) // 2

        bottom_y = max(y + 10, self.sidebar_rect.bottom - 136)

        self.buttons["save_map"].set_rect((x, bottom_y, half, 34))

        self.buttons["load_map"].set_rect((x + half + 8, bottom_y, half, 34))

        self.buttons["run"].set_rect((x, bottom_y + 44, half, 34))

        self.buttons["pause"].set_rect((x + half + 8, bottom_y + 44, half, 34))

        self.buttons["reset"].set_rect((x, bottom_y + 88, button_width, 34))

    def handle_button(self, action):

        if action == "mode":

            self.developer_mode = not self.developer_mode

            self.layout_controls()

            if self.developer_mode:

                self.status_message = ("Developer mode")

            else:

                self.status_message = ("Normal mode")


        elif action == "run":

            if (len(self.simulation.drones) == 0):

                self.spawn_drones()

            if (len(self.simulation.drones) > 0):

                self.running_simulation = (True)

                self.status_message = ("Running")


        elif action == "pause":

            self.running_simulation = (False)

            self.status_message = ("Paused")


        elif action == "save_map":

            self.save_map()


        elif action == "load_map":

            self.load_map()


        elif action.startswith("toggle_"):

            visual_key = action.replace("toggle_", "")

            if visual_key in self.visuals:

                self.visuals[visual_key] = not self.visuals[visual_key]

                state = ("ON" if self.visuals[visual_key] else "OFF")

                self.status_message = (f"{visual_key.replace('_', ' ').title()}: {state}")


        elif action == "reset":

            self.reset()


        else:

            if (self.placement_mode == action):

                self.placement_mode = (None)

            else:

                self.placement_mode = (action)

            self.painting = None


            if (self.placement_mode is None):

                self.status_message = ("Ready")

            elif (self.placement_mode in PAINT_MODES):

                self.status_message = ("Drag to paint, " "right-drag to erase")

            else:

                self.status_message = ("Click environment " "to place target")

    def handle_event(self, event):

        if (event.type == pygame.QUIT):

            return False

        if (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):

            return False


        # -----------------------------
        # Numeric fields
        # -----------------------------

        for (key, field) in self.fields.items():

            if field.handle_event(event):

                self.apply_settings(key)


        # -----------------------------
        # Mouse press
        # -----------------------------

        if (event.type == pygame.MOUSEBUTTONDOWN and event.button in (1, 3)):

            if (event.button == 1):

                for key, button in (self.buttons.items()):

                    if key not in self.visible_button_keys:

                        continue

                    if button.clicked(event.pos):

                        self.handle_button(button.action)

                        return True


            if (self.environment_rect.collidepoint(event.pos)):

                if (self.placement_mode in PAINT_MODES):

                    self.painting = ("add" if event.button == 1 else "erase")

                    self.last_paint_pos = (None)

                    self.paint_stroke(event.pos)


                elif (self.placement_mode == "target" and event.button == 1):

                    self.place_target(event.pos)


        # -----------------------------
        # Mouse movement
        # -----------------------------

        elif (event.type == pygame.MOUSEMOTION and self.painting):

            if (self.environment_rect.collidepoint(event.pos)):

                self.paint_stroke(event.pos)

            else:

                self.last_paint_pos = (None)


        # -----------------------------
        # Stop painting
        # -----------------------------

        elif (event.type == pygame.MOUSEBUTTONUP and event.button in (1, 3)):

            self.painting = None

            self.last_paint_pos = (None)


        # -----------------------------
        # Mouse wheel brush
        # -----------------------------

        elif (event.type == pygame.MOUSEWHEEL and self.placement_mode in PAINT_MODES):

            self.adjust_brush(0.5 * event.y)


        # -----------------------------
        # Keyboard brush
        # -----------------------------

        elif (event.type == pygame.KEYDOWN and self.placement_mode in PAINT_MODES):

            active_field = any(field.active for field in self.fields.values())

            if not active_field:

                if (event.unicode == "["):

                    self.adjust_brush(-0.5)

                elif (event.unicode == "]"):

                    self.adjust_brush(0.5)

        return True

    def apply_live_settings(self):

        for drone in (self.simulation.drones):

            drone.sensing_radius = float(self.fields["sense"].value)

            drone.neighbour_radius = float(self.fields["neighbour"].value)

            drone.communication_radius = float(self.fields["comm"].value)

            drone.separation_weight = float(self.fields["separation"].value)

            drone.alignment_weight = float(self.fields["alignment"].value)

            drone.cohesion_weight = float(self.fields["cohesion"].value)

            drone.goal_weight = float(self.fields["goal"].value)

            drone.obstacle_weight = float(self.fields["obstacle_force"].value)

            drone.restricted_weight = float(self.fields["restricted_force"].value)

    def run(self):

        alive = True

        try:

            while alive:

                for event in (pygame.event.get()):

                    alive = (self.handle_event(event))

                    if not alive:
                        break


                # -----------------------------
                # Run simulation
                # -----------------------------

                if (self.running_simulation):

                    self.apply_live_settings()

                    self.simulation.update()


                # -----------------------------
                # Draw everything
                # -----------------------------

                self.draw_frame()

                self.clock.tick(int(self.config["gui"]["target_fps"]))


        # -----------------------------
        # Clean Ctrl+C exit
        # -----------------------------

        except KeyboardInterrupt:

            pass


        # -----------------------------
        # Clean Pygame exit
        # -----------------------------

        finally:

            pygame.quit()
