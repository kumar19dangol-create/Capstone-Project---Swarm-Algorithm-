import json
import math
import tkinter as tk
from tkinter import filedialog

from .settings import *


class MapEditorMixin:

    def world_to_screen(self, x, y):

        px = (self.environment_rect.x + (x / WORLD_WIDTH) * self.environment_rect.width)

        py = (self.environment_rect.bottom - (y / WORLD_HEIGHT) * self.environment_rect.height)

        return (int(px), int(py))

    def screen_to_world(self, px, py):

        x = ((px - self.environment_rect.x) / self.environment_rect.width * WORLD_WIDTH)

        y = ((self.environment_rect.bottom - py) / self.environment_rect.height * WORLD_HEIGHT)

        return (max(0.0, min(WORLD_WIDTH, x)), max(0.0, min(WORLD_HEIGHT, y)))

    def save_map(self):

        root = tk.Tk()

        root.withdraw()

        root.attributes("-topmost", True)

        file_path = filedialog.asksaveasfilename(title="Save Map", defaultextension=".json", filetypes=[("JSON Map", "*.json")])

        root.destroy()

        if not file_path:

            self.status_message = ("Save cancelled")

            return

        map_data = {
            "world": {"width": self.world.width, "height": self.world.height},
            "obstacles": [{"x": float(obstacle["x"]), "y": float(obstacle["y"]), "width": float(obstacle["width"]), "height": float(obstacle["height"])} for obstacle in self.world.obstacles],
            "restricted_zones": [{"x": float(zone["x"]), "y": float(zone["y"]), "width": float(zone["width"]), "height": float(zone["height"])} for zone in self.world.restricted_zones],
            "targets": [{"x": float(target["x"]), "y": float(target["y"])} for target in self.world.targets]
        }

        try:

            with open(file_path, "w", encoding="utf-8") as file:

                json.dump(map_data, file, indent=4)

            self.status_message = ("Map saved")

        except (OSError, TypeError, ValueError):

            self.status_message = ("Could not save map")

    def load_map(self):

        root = tk.Tk()

        root.withdraw()

        root.attributes("-topmost", True)

        file_path = filedialog.askopenfilename(title="Load Map", filetypes=[("JSON Map", "*.json")])

        root.destroy()

        if not file_path:

            self.status_message = ("Load cancelled")

            return

        try:

            with open(file_path, "r", encoding="utf-8") as file:

                map_data = json.load(file)

            world_data = map_data.get("world", {})

            if (float(world_data.get("width", WORLD_WIDTH)) != WORLD_WIDTH or float(world_data.get("height", WORLD_HEIGHT)) != WORLD_HEIGHT):

                self.status_message = ("Map size does not match current world")

                return

            obstacles = map_data.get("obstacles", [])

            restricted_zones = map_data.get("restricted_zones", [])

            targets = map_data.get("targets", [])

            self.reset()

            for obstacle in obstacles:

                x = float(obstacle["x"])
                y = float(obstacle["y"])
                width = float(obstacle.get("width", 1.0))
                height = float(obstacle.get("height", 1.0))

                if (width == 1.0 and height == 1.0 and x.is_integer() and y.is_integer()):

                    self.add_obstacle_cell(int(x), int(y))

                else:

                    self.world.add_obstacle(x=x, y=y, width=width, height=height)

            for zone in restricted_zones:

                x = float(zone["x"])
                y = float(zone["y"])
                width = float(zone.get("width", 1.0))
                height = float(zone.get("height", 1.0))

                if (width == 1.0 and height == 1.0 and x.is_integer() and y.is_integer()):

                    self.add_restricted_cell(int(x), int(y))

                else:

                    self.world.add_restricted_zone(x=x, y=y, width=width, height=height)

            for target in targets:

                self.world.add_target(float(target["x"]), float(target["y"]))

            self.running_simulation = (False)

            self.status_message = (f"Map loaded: {len(obstacles)} obstacles, {len(restricted_zones)} restricted, {len(targets)} targets")

        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):

            self.status_message = ("Could not load map")

    def add_obstacle_cell(self, cell_x, cell_y):

        if (cell_x, cell_y) in self.obstacle_cells:

            return

        obstacle = {"x": float(cell_x), "y": float(cell_y), "width": 1.0, "height": 1.0}

        self.obstacle_cells[(cell_x, cell_y)] = obstacle

        self.world.obstacles.append(obstacle)

    def add_restricted_cell(self, cell_x, cell_y):

        if (cell_x, cell_y) in self.restricted_cells:

            return

        self.restricted_cells.add((cell_x, cell_y))

        self.world.add_restricted_zone(x=float(cell_x), y=float(cell_y), width=1.0, height=1.0)

    def brush_cells(self, position):

        x, y = (self.screen_to_world(*position))

        radius = (self.brush_radius)

        cells = []

        for cell_x in range(int(x - radius) - 1, int(x + radius) + 2):

            for cell_y in range(int(y - radius) - 1, int(y + radius) + 2):

                if not (0 <= cell_x < WORLD_WIDTH and 0 <= cell_y < WORLD_HEIGHT):

                    continue

                distance = math.hypot(cell_x + 0.5 - x, cell_y + 0.5 - y)

                if (distance <= radius):

                    cells.append((cell_x, cell_y))

        return cells

    def paint_at(self, position):

        erase = (self.painting == "erase")

        cells = (self.brush_cells(position))


        # -----------------------------
        # Obstacles
        # -----------------------------

        if (self.placement_mode == "obstacle"):

            if erase:

                for cell in cells:

                    self.obstacle_cells.pop(cell, None)

                self.world.obstacles = (list(self.obstacle_cells.values()))

            else:

                for (cell_x, cell_y) in cells:

                    self.add_obstacle_cell(cell_x, cell_y)


        # -----------------------------
        # Restricted zones
        # -----------------------------

        elif (self.placement_mode == "restricted"):

            if erase:

                for cell in cells:

                    self.restricted_cells.discard(cell)

                self.world.restricted_zones = [

                    {"x": float(cell_x), "y": float(cell_y), "width": 1.0, "height": 1.0}

                    for (cell_x, cell_y)
                    in self.restricted_cells
                ]

            else:

                for (cell_x, cell_y) in cells:

                    self.add_restricted_cell(cell_x, cell_y)

    def paint_stroke(self, position):

        if (self.last_paint_pos is None):

            self.paint_at(position)

        else:

            x0, y0 = (self.last_paint_pos)

            x1, y1 = (position)

            steps = max(1, int(math.hypot(x1 - x0, y1 - y0) / 3))

            for index in range(1, steps + 1):

                amount = (index / steps)

                position_x = (x0 + (x1 - x0) * amount)

                position_y = (y0 + (y1 - y0) * amount)

                self.paint_at((position_x, position_y))

        self.last_paint_pos = (position)

    def adjust_brush(self, amount):

        self.brush_radius = max(BRUSH_MIN, min(BRUSH_MAX, self.brush_radius + amount))

        field = (self.fields["brush"])

        field.value = (self.brush_radius)

        field.text = (field.format_value(self.brush_radius))

        self.status_message = (f"Brush size: " f"{self.brush_radius:g}")

    def place_target(self, position):

        x, y = (self.screen_to_world(*position))

        self.world.add_target(x, y)

        self.status_message = ("Target added")

        self.placement_mode = (None)
