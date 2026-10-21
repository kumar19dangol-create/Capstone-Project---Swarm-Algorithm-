import numpy as np


class CommunicationMixin:

    def communicate_drones(self):

        for drone in (self.drones):

            drone.communicate(self.drones)

    def get_communication_network(self):

        network = {}
        for drone in (self.drones):

            neighbours = (drone.get_communication_neighbours(self.drones))
            network[drone.id] =[neighbour.id for neighbour in neighbours]

        return network

    def get_communication_components(self):

        network = (self.get_communication_network())
        components = []
        visited = set()
        for drone in (self.drones):

            if (drone.id in visited):

                continue

            component = []
            stack =[drone.id]
            while (len(stack) > 0):

                current_id = (stack.pop())
                if (current_id in visited):

                    continue

                visited.add(current_id)
                component.append(current_id)
                for neighbour_id in (network[current_id]):

                    if (neighbour_id not in visited):

                        stack.append(neighbour_id)

            components.append(component)

        return components

    def get_main_component(self):

        components = (self.get_communication_components())
        if (len(components) == 0):

            return[]

        main_component = max(components, key=len)
        return main_component

    def is_swarm_connected(self):

        components = (self.get_communication_components())
        return (len(components) <= 1)

    def get_disconnected_drones(self):

        main_component = (self.get_main_component())
        disconnected_drones = []
        for drone in (self.drones):

            if (drone.id not in main_component):

                disconnected_drones.append(drone)

        return disconnected_drones

    def get_main_swarm_drones(self):

        main_component = (self.get_main_component())
        main_drones = []
        for drone in (self.drones):

            if (drone.id in main_component):

                main_drones.append(drone)

        return main_drones

    def get_nearest_main_swarm_drone(self, drone):

        main_drones = (self.get_main_swarm_drones())
        nearest_drone = None
        nearest_distance = (float("inf"))
        for other_drone in (main_drones):

            if (other_drone.id == drone.id):

                continue

            distance = (np.linalg.norm(other_drone.position - drone.position))
            if (distance < nearest_distance):

                nearest_distance = (distance)
                nearest_drone = (other_drone)

        return nearest_drone

    def get_reconnection_frontier(self, drone):

        frontier_positions = (self.get_frontier_positions())
        if (len(frontier_positions) == 0):

            return None

        nearest_main_drone = (self.get_nearest_main_swarm_drone(drone))
        if (nearest_main_drone is None):

            return None

        best_frontier = None
        best_cost = (float("inf"))
        for frontier in (frontier_positions):

            drone_distance = (np.linalg.norm(frontier - drone.position))
            communication_distance = (np.linalg.norm(frontier - nearest_main_drone.position))
            cost = (drone_distance + communication_distance * 2.0)
            if (cost < best_cost):

                best_cost = (cost)
                best_frontier = (frontier)

        if (best_frontier is None):

            return None

        return (best_frontier.copy())

    def maintain_swarm_connectivity(self):

        # Do not change goals while

        # the whole swarm is paused

        if (self.swarm_pause_timer > 0.0):

            return

        if (self.is_swarm_connected()):

            return

        disconnected_drones = (self.get_disconnected_drones())
        for drone in (disconnected_drones):

            # Target task has higher priority

            if (drone.target_assignment is not None):

                continue

            reconnection_frontier = (self.get_reconnection_frontier(drone))
            if (reconnection_frontier is not None):

                drone.exploration_goal = (reconnection_frontier)
