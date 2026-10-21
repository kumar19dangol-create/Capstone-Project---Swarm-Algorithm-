# Autonomous Drone Swarm Exploration Simulator

A Python-based 2D simulation for developing, visualising, and evaluating
autonomous multi-drone swarm behaviour.

The project models a swarm of drones exploring an initially unknown
environment using a discretised belief grid, frontier-based exploration,
local Boids-inspired swarm behaviour, drone-to-drone communication,
connectivity-aware goal selection, obstacle and restricted-zone
avoidance, target detection, and performance metrics. A full-screen
Pygame interface is provided for creating environments, changing swarm
parameters, running experiments, and visualising the internal behaviour
of the swarm.

## Project Purpose

The simulator is designed as the technical engine for a capstone project
investigating autonomous swarm exploration and mapping. The main
objective is to allow multiple drones to explore an environment
autonomously while balancing several competing requirements:

-   cover unknown areas efficiently;
-   avoid obstacles and restricted areas;
-   prevent drones from crowding each other;
-   maintain coordinated local swarm behaviour;
-   distribute exploration goals between drones;
-   consider communication connectivity when selecting goals;
-   detect and respond to targets; and
-   measure exploration performance.

The simulation is entirely software-based. It does not control physical
drones.

## Main Technologies

-   **Python**
-   **Pygame** --- graphical interface, visualisation, controls, and
    simulation loop
-   **NumPy** --- vector calculations, distance calculations, swarm
    behaviour, and optimised frontier/connectivity calculations
-   **Tkinter** --- native file dialogs for saving and loading maps
-   **JSON** --- project configuration and saved map format

## Project Structure

``` text
SwarmAlgorithmSimulation_Polished/
│
├── main.py
├── config.json
├── config_loader.py
├── README.md
├── README_STRUCTURE.txt
│
├── gui/
│   ├── __init__.py
│   ├── app.py
│   ├── components.py
│   ├── map_editor.py
│   ├── renderer.py
│   └── settings.py
│
├── maps/
│
├── models/
│   ├── __init__.py
│   ├── belief.py
│   ├── drone.py
│   └── world.py
│
└── simulation/
    ├── __init__.py
    ├── communication.py
    ├── exploration.py
    ├── metrics.py
    ├── simulation.py
    └── target_manager.py
```

Python cache folders such as `__pycache__/` and compiled `.pyc` files
are runtime-generated files and should not be committed to the
repository.

## How the Simulation Works

When **RUN** is selected, the configured number of drones are placed at
random safe positions in the environment. The simulation then repeatedly
performs sensing, target handling, communication, exploration planning,
swarm movement, avoidance, and position updates.

The overall process is:

``` text
Start simulation
      ↓
Spawn drones at safe positions
      ↓
Sense nearby environment
      ↓
Update the belief grid
      ↓
Detect targets / known hazards
      ↓
Exchange exploration-assignment information
      ↓
Identify explored-to-unknown frontiers
      ↓
Evaluate and assign frontier goals
      ↓
Move toward individual goals
      +
Separation / Alignment / Cohesion
      +
Obstacle / Restricted avoidance
      ↓
Update positions and world boundaries
      ↓
Repeat
```

Sensing and movement happen continuously. More expensive high-level
planning is intentionally performed less frequently to improve
performance.

## Environment and Belief Grid

The simulator separates the **ground-truth world** from what the swarm
currently knows.

The `World` stores the real environment, including obstacles, restricted
zones and targets. The `BeliefGrid` stores the swarm's discovered
representation of that environment.

Belief-grid cells can represent:

-   `UNKNOWN`
-   `EXPLORED`
-   `OBSTACLE`
-   `TARGET`
-   `RESTRICTED`

At the start of a mission, most of the belief grid is unknown. As drones
move, cells inside their sensing radius are revealed. Obstacles,
restricted areas and targets become known when they fall within sensing
range.

This creates a distinction between what exists in the simulated world
and what the autonomous system has actually discovered.

## Frontier-Based Autonomous Exploration

The project uses frontier-based exploration.

A **frontier** is an explored cell next to at least one unknown cell.
Frontiers therefore represent useful boundaries where a drone can move
to reveal additional unknown space.

The exploration process:

1.  obtains the current frontier cells from the belief grid;
2.  converts them into world positions;
3.  keeps valid existing exploration goals;
4.  prevents new goals from being assigned too close to existing goals;
5.  calculates travel, communication, and predicted-connectivity costs
    for available frontiers;
6.  chooses the lowest-cost suitable frontier for each drone that
    requires a goal; and
7.  stores it as that drone's `exploration_goal`.

Goals are therefore not generated randomly.

### Frontier spacing

`frontier_spacing` prevents several drones from being assigned almost
the same exploration location. In the current simulation logic this is
set to `8.0` world units.

### Goal completion

When a drone moves within `goal_tolerance` of its exploration goal, the
goal is released. The planner can then assign a new frontier.

The current simulation logic uses a tolerance of `2.0` world units.

## Communication and Connectivity

Communication is separate from physical Boids interaction.

Each drone has a **communication radius**. Drones within communication
range are direct communication neighbours.

The communication system is used for two main purposes.

### Sharing exploration assignments

`Drone.communicate()` exchanges exploration-goal assignments with
communication neighbours. A drone also receives assignments that its
neighbours already know, allowing assignment information to propagate
through connected drones.

The shared information is stored in `known_assignments`.

### Building the communication network

The simulator builds a graph where:

-   each drone is a node;
-   a direct communication link exists when two drones are within
    communication range.

This graph is used to determine connected components, the largest/main
component, disconnected drones, and whether the swarm is currently
connected.

Because connectivity can be multi-hop, two drones do not need to
communicate directly to belong to the same connected network. For
example:

``` text
D1 ---- D2 ---- D3 ---- D4
```

D1 and D4 may be outside direct communication range, but they are still
part of the same connected component through D2 and D3.

## How Communication Influences Goal Selection

The communication radius does **not** limit how far away an exploration
goal may be. It describes drone-to-drone communication range, not
drone-to-goal range.

When a candidate frontier is evaluated, the exploration planner
currently combines:

``` text
Total cost =
    travel distance
  + communication cost
  + predicted connectivity cost
```

### Travel distance

Closer frontiers receive a lower travel-distance cost.

### Communication cost

For each candidate frontier, the planner checks the distance from that
frontier to the nearest other drone. If the predicted position is within
the candidate drone's communication radius, there is no
communication-distance penalty. Distance beyond that radius adds
additional cost.

### Predicted connectivity cost

The planner also predicts the communication graph if the candidate drone
were moved to the frontier while other drones are represented by their
target goal, exploration goal, or current position.

If the predicted network contains multiple disconnected components, the
candidate receives an additional connectivity penalty.

This means the system tries to balance exploration distance with
communication connectivity instead of enforcing a hard rule that every
goal must remain inside communication range.

## Connectivity Repair

`simulation/communication.py` also contains reactive connectivity
handling.

If the current communication graph is split, the largest connected
component is treated as the main swarm. Drones outside that component
are considered disconnected.

For a disconnected drone that is not currently handling a target, the
simulator searches the available frontiers and selects a
reconnection-oriented frontier using both:

-   distance from the disconnected drone; and
-   distance from the frontier to the nearest drone in the main
    component.

The selected reconnection frontier is assigned as that drone's
exploration goal.

## Local Swarm Behaviour

The physical swarm behaviour is based on three classic Boids-style
behaviours:

### Separation

Separation prevents nearby drones from becoming too close. Drones inside
the separation distance generate a movement correction away from each
other.

### Alignment

Alignment encourages a drone to move in a similar direction to drones
inside its neighbour radius.

### Cohesion

Cohesion calculates the centre of nearby neighbours and creates a
movement influence toward that local centre.

Cohesion does **not** mean all drones first travel to one global centre
before exploration starts. It is a continuously recalculated local
movement influence.

## Neighbour Radius vs Communication Radius

These parameters serve different purposes.

**Neighbour radius** determines which drones affect Separation,
Alignment and Cohesion.

**Communication radius** determines which drones can directly exchange
information and form communication links.

For example, if:

``` text
Neighbour radius     = 15
Communication radius = 20
```

a drone 10 units away can influence Boids behaviour and communicate. A
drone 18 units away can communicate but does not influence the local
Boids calculation. A drone 25 units away does neither directly.

This separation allows drones to spread across the map without requiring
every communicating drone to remain in one tightly packed group.

## Six Adjustable Movement Behaviours

The Developer interface exposes six movement weights.

### 1. Separation Weight

Controls the influence of collision/crowding avoidance between nearby
drones.

Higher values make separation more influential. A value of `0` disables
the weighted separation contribution.

### 2. Alignment Weight

Controls how strongly a drone follows the movement direction of its
local neighbours.

### 3. Cohesion Weight

Controls how strongly a drone is influenced toward the centre of its
local neighbours.

### 4. Goal Weight

Controls how strongly the current exploration or target goal guides the
drone's direction.

The implementation uses the goal weight to scale a gentle steering
influence toward the selected goal rather than instantly pointing the
drone directly at it.

### 5. Obstacle Weight

Controls the strength of obstacle avoidance once an obstacle is detected
within the avoidance behaviour's working range.

### 6. Restricted Weight

Controls the strength of avoidance for restricted zones.

These weights influence movement differently and are not intended to be
interpreted as identical physical forces.

## Obstacle and Restricted-Zone Avoidance

Drones search the known belief grid for nearby obstacle and restricted
cells.

When a relevant hazard is close enough, the drone calculates an
avoidance direction away from it and adjusts its movement. Obstacle and
restricted-zone avoidance have separate weights so their responses can
be tuned independently.

A drone can only react to hazards represented in its current belief
grid, meaning the sensing process is important to avoidance.

## Target Detection and Response

Targets exist in the ground-truth world but are initially unknown to the
swarm.

When a target enters a drone's sensing radius:

1.  the target becomes detected;
2.  its position is marked in the belief grid;
3.  the nearest currently available drone is assigned to it;
4.  the assigned drone receives a `target_goal`;
5.  target movement takes priority over normal frontier exploration;
6.  when the assigned drone reaches the target, the target is marked as
    found; and
7.  the entire swarm pauses for the configured target-pause period.

The current simulation logic uses a target tolerance of `1.0` world unit
and a swarm pause duration of `3.0` simulation seconds.

After the global pause finishes, normal exploration can continue.

## Performance Metrics

The current metrics system records:

### Map coverage

Coverage is calculated as:

``` text
known belief-grid cells / total belief-grid cells × 100
```

Any non-`UNKNOWN` cell counts as known, including explored, obstacle,
target and restricted cells.

### Target find time

The first-target timer is frozen when any target is first **detected**,
not when the assigned drone physically reaches it.

### Map coverage time

The coverage timer is frozen when map coverage first reaches the
configured completion threshold. The current simulation logic uses
`95%`.

## Performance Optimisation

Movement and sensing need to remain responsive, but frontier and
connectivity calculations are more expensive.

The simulation therefore separates the fast movement loop from the
high-level planning loop.

Movement, sensing, target checks and avoidance continue during normal
simulation updates. Expensive operations such as frontier validation,
connectivity repair and frontier assignment are performed at a lower
frequency.

The current planning interval is `12` update calls. At a 60 FPS GUI
target this corresponds to roughly five high-level planning cycles per
second when the application maintains the target frame rate.

Planning is also triggered immediately when a non-target drone has no
exploration goal, which prevents unnecessary delay at startup or after a
goal is completed.

NumPy vectorisation is used in frontier evaluation and
predicted-connectivity calculations to reduce repeated Python loops.

## Graphical User Interface

The project uses a full-screen Pygame interface.

The GUI provides:

-   simulation RUN/PAUSE/RESET controls;
-   drone-count control;
-   environment editing;
-   obstacle painting;
-   restricted-zone painting;
-   target placement;
-   adjustable brush size;
-   map saving and loading;
-   Normal and Developer modes;
-   live swarm-parameter editing;
-   performance/status information; and
-   optional internal visualisations.

Press `Esc` to exit the application.

## Normal and Developer Modes

### Normal Mode

Normal Mode keeps the interface focused on the main simulation and map
editing.

### Developer Mode

Developer Mode exposes additional swarm parameters and diagnostic
visualisations. Depending on the rendering toggles, the simulator can
display:

-   sensing-radius circles;
-   neighbour-radius circles;
-   communication-radius circles;
-   communication links;
-   neighbour links; and
-   frontier/exploration goals.

These visualisations are intended to make the internal autonomous
behaviour easier to inspect and tune.

## Map Editor

The GUI can create test environments without modifying source code.

The editor supports:

-   painting obstacle cells;
-   painting restricted cells;
-   erasing painted cells;
-   changing brush radius; and
-   placing targets.

The map uses simulation/world coordinates even though it is scaled to
fill the available GUI environment panel.

## Saving and Loading Maps

Maps are stored as JSON.

Saved maps contain:

-   world width and height;
-   obstacles;
-   restricted zones; and
-   target positions.

Drone state, belief-grid progress, discovered state, timers and current
exploration goals are not saved.

When a map is loaded, the simulator checks that its world dimensions
match the current environment before rebuilding the map.

The `maps/` directory is provided as a convenient location for saved map
files, although the file dialog allows another location to be selected.

## Configuration

`config.json` stores the intended central configuration for the project.
It currently contains sections for:

-   environment;
-   swarm;
-   movement forces;
-   exploration;
-   targets;
-   metrics;
-   optimisation;
-   spawning;
-   map editor;
-   GUI;
-   input limits; and
-   rendering.

The current checked-in configuration includes values such as:

``` text
World size             200 × 100
Default drones         20
Sensing radius         5
Neighbour radius       15
Communication radius   20
Desired speed          1
Maximum speed          3
Target GUI FPS         60
```

### Important implementation note

Not every value in `config.json` is currently wired into every
subsystem.

The GUI directly loads values such as drone count, sensing radius,
neighbour radius, communication radius, the six force weights, initial
velocity, rendering defaults, editor values and target FPS from
`config.json`.

However, some simulation/planning values are still defined directly in
the Python implementation. For example, the current `Simulation` class
directly sets goal tolerance, frontier spacing, target tolerance, target
pause duration, coverage target and planning interval. The current
exploration and reconnection code also contains fixed penalty
multipliers.

Therefore, changing every value in `config.json` does not yet guarantee
that the corresponding simulation behaviour changes. Further refactoring
can make the JSON configuration the single source of truth for all
parameters.

## File-by-File Explanation

### `main.py`

Minimal application entry point. It imports `SwarmGUI`, creates the
application, and starts the GUI run loop.

### `config.json`

Stores user-adjustable and project configuration values for the
environment, swarm, force weights, exploration, target behaviour,
metrics, optimisation, spawning, editor, GUI, limits and rendering.

### `config_loader.py`

Loads `config.json` and exposes a small `cfg(section, key)` helper. The
current GUI also reads `config.json` directly.

### `README_STRUCTURE.txt`

A short development note describing the refactored project layout and
the intended responsibility of each module.

### `models/world.py`

Represents the ground-truth simulation environment. It stores world
dimensions, obstacles, targets and restricted zones. It also handles
world-boundary checks and boundary reflection for drones.

### `models/belief.py`

Implements the discretised belief grid used by the autonomous system. It
converts between world and grid coordinates, marks explored and
discovered cells, and detects frontier cells between explored and
unknown space.

### `models/drone.py`

Contains the state and low-level behaviour of an individual drone. This
includes position, velocity, sensing/interaction parameters,
communication neighbours, known assignments, Separation, Alignment,
Cohesion, goal steering, obstacle avoidance, restricted-zone avoidance,
and minimum separation enforcement.

### `simulation/simulation.py`

The central simulation orchestrator. It owns the world, belief grid,
drones and simulation time, performs sensing, calls target and
communication logic, triggers high-level planning, calculates movement
decisions, updates drone positions, applies world boundaries, and
coordinates the complete update cycle.

It combines the specialised simulation modules through mixins:

``` python
Simulation(
    TargetManagerMixin,
    ExplorationMixin,
    CommunicationMixin,
    MetricsMixin
)
```

### `simulation/exploration.py`

Implements frontier-based exploration. It obtains frontier positions,
validates existing goals, detects completed goals, calculates
communication and predicted-connectivity costs, enforces spacing between
assigned frontiers, and distributes exploration goals to drones.

The predicted-connectivity calculation includes a vectorised
implementation to reduce the cost of evaluating many candidate
frontiers.

### `simulation/communication.py`

Implements swarm communication and connectivity analysis. It triggers
drone communication, constructs the communication graph, identifies
connected components, finds the main component, identifies disconnected
drones, and provides reactive reconnection goal selection.

### `simulation/target_manager.py`

Handles target tasking. It detects which targets require assignments,
assigns the nearest available drone, checks when assigned drones reach
targets, marks targets as found, and controls the temporary whole-swarm
pause after a target is reached.

### `simulation/metrics.py`

Calculates map coverage and records the first target-detection time and
the time at which the required map-coverage threshold is reached.

### `gui/app.py`

Main GUI controller. It creates the full-screen Pygame window, loads GUI
configuration, creates numeric fields and buttons, builds/reset the
simulation, safely spawns drones, applies live settings, handles user
input, switches GUI modes, and runs the main application loop.

### `gui/components.py`

Contains reusable GUI widgets, currently including numeric input fields
and buttons.

### `gui/map_editor.py`

Implements map editing and coordinate conversion. It handles
obstacle/restricted-zone painting, brush strokes, erasing, target
placement, and JSON map save/load operations.

### `gui/renderer.py`

Handles visual rendering of the simulation, including the environment,
belief grid, drones, goals, communication/neighbour information, sidebar
controls, metrics and status information.

### `gui/settings.py`

Stores GUI/world constants and colour definitions used by the interface
and renderer.

### `gui/__init__.py`

Exports the main `SwarmGUI` class for convenient importing.

### `simulation/__init__.py`

Exports the `Simulation` class.

### `models/__init__.py`

Marks the models directory as a Python package.

### `maps/`

Suggested directory for JSON maps created with the map editor.

## Installation

### 1. Clone the repository

``` bash
git clone https://github.com/kumar19dangol-create/Capstone-Project---Swarm-Algorithm-.git
cd Capstone-Project---Swarm-Algorithm-
```

### 2. Create a virtual environment (recommended)

Windows:

``` bash
python -m venv .venv
.venv\Scripts\activate
```

### 3. Install dependencies

``` bash
pip install numpy pygame
```

Tkinter is normally included with standard Windows Python installations.

## Running the Simulator

Run the application from the project root:

``` bash
python main.py
```

The simulator opens in full-screen mode.

Press `Esc` to exit.

## Basic Usage

1.  Start the application.
2.  Paint obstacles or restricted areas if required.
3.  Add one or more targets if required.
4.  Select the desired number of drones.
5.  Adjust swarm parameters if required.
6.  Switch to Developer Mode if diagnostic controls are needed.
7.  Press **RUN**.
8.  Observe autonomous sensing, frontier assignment, swarm movement,
    communication and target behaviour.
9.  Use **PAUSE** to stop updates temporarily.
10. Use **RESET** to start a new simulation while retaining the designed
    environment as implemented by the GUI reset/rebuild behaviour.

## Current Design Characteristics

The simulator is intentionally based on distributed/local behaviour
rather than commanding every drone to move as one rigid group.

Each drone:

-   senses only its local environment;
-   interacts physically with drones inside its neighbour radius;
-   communicates directly with drones inside its communication radius;
-   can receive an individual exploration goal;
-   moves toward that goal while local swarm behaviours modify its
    direction;
-   avoids discovered hazards; and
-   can be reassigned to a detected target.

The wider swarm emerges from these local interactions combined with the
higher-level frontier and connectivity planner.

## Current Limitations and Future Work

Potential future improvements include:

-   making `config.json` the single source of truth for all simulation
    parameters;
-   improving global frontier assignment to reduce unnecessary cross-map
    travel;
-   improving reconnection behaviour so connectivity repair does not
    unnecessarily overwrite useful exploration goals;
-   adding more detailed connectivity and travel metrics;
-   improving the distinction between 95% completion time and true 100%
    map coverage in the GUI;
-   evaluating alternative frontier clustering or assignment algorithms;
-   adding path planning around complex obstacles if required;
-   profiling larger swarm sizes; and
-   systematic parameter optimisation and experimental evaluation.

## Research / Experimental Use

The Developer Mode and configurable parameters are intended to support
controlled experiments. Parameters such as neighbour radius,
communication radius and the six movement weights can be varied to
observe their effect on:

-   map coverage;
-   exploration time;
-   swarm dispersion;
-   communication connectivity;
-   target detection;
-   movement stability; and
-   obstacle/restricted-zone behaviour.

For meaningful experiments, record the parameter set and map used for
each run because random initial drone placement can affect the outcome.

## Repository Hygiene

The repository should exclude generated Python cache files and local
development files. A suitable `.gitignore` should include at least:

``` gitignore
__pycache__/
*.py[cod]
.venv/
venv/
.vscode/
.idea/
.pytest_cache/
.mypy_cache/
.ruff_cache/
*.log
```

## Status

This repository contains the current working autonomous swarm simulation
prototype used for the capstone project. The system includes the core
simulation engine, GUI, map editor, frontier exploration, swarm
behaviour, communication/connectivity logic, target handling and initial
performance metrics.
