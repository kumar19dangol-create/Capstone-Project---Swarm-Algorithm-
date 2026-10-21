REFACTORED PROJECT STRUCTURE

main.py                 Starts the application only.
gui/app.py              GUI state, events, controls and main loop.
gui/components.py       NumericField and Button widgets.
gui/map_editor.py       Map painting plus save/load logic.
gui/renderer.py         All environment/sidebar/status drawing.
gui/settings.py         GUI colours, dimensions and constants.
simulation/simulation.py Core simulation lifecycle and update orchestration.
simulation/exploration.py Frontier exploration and goal assignment.
simulation/communication.py Communication/connectivity/reconnection.
simulation/target_manager.py Target assignment and swarm pause behavior.
simulation/metrics.py   Coverage and timing metrics.
models/drone.py         Drone movement and Boids behavior.
models/belief.py        Belief grid and frontier detection.
models/world.py         Ground-truth world objects and boundaries.
maps/                   Suggested location for saved map JSON files.

Run from this folder with: python main.py
