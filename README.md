# Autonomous Drone Swarm Exploration Simulator

## Overview

The Autonomous Drone Swarm Exploration Simulator is a Python-based 2D simulation developed to model, visualise, and evaluate autonomous multi-drone swarm behaviour. The main purpose of the project is to enable multiple drones to move effectively and efficiently as a coordinated swarm to explore and map an initially unknown environment while locating targets.

To achieve this, the swarm must be able to:

- Explore and map unknown areas efficiently.
- Coordinate movement between multiple drones.
- Distribute exploration goals across the swarm.
- Avoid obstacles and restricted areas.
- Prevent drones from crowding or colliding with each other.
- Maintain communication and connectivity where possible.
- Detect and respond to targets autonomously.
- Measure exploration and target-finding performance.

The simulator provides a Pygame-based graphical interface for creating environments, configuring swarm parameters, running simulations, and visualising the behaviour and performance of the swarm.


## Key Features

- **Autonomous Swarm Exploration** – Multiple drones autonomously explore and map an initially unknown environment.
- **Discretised Belief-Grid Mapping** – Builds and updates a shared representation of explored and unknown areas as the swarm moves.
- **Frontier-Based Exploration** – Identifies useful exploration frontiers and distributes goals between drones.
- **Swarm Coordination** – Uses Separation, Alignment, and Cohesion to support coordinated multi-drone movement.
- **Communication & Connectivity** – Supports drone-to-drone communication, multi-hop connectivity, and connectivity-aware exploration.
- **Obstacle & Restricted-Area Avoidance** – Drones detect and respond to obstacles and restricted areas while exploring.
- **Autonomous Target Detection & Response** – Targets can be discovered during exploration and assigned to an available drone.
- **Configurable Swarm Behaviour** – Important swarm parameters and movement behaviours can be adjusted for testing and evaluation.
- **Interactive Environment Editor** – Obstacles, restricted areas, and targets can be added directly through the GUI, with support for saving and loading maps.
- **Normal & Developer Modes** – Provides a simplified simulation view and a developer view with additional parameters and diagnostic visualisations.
- **Performance Metrics** – Measures map coverage, coverage time, and target-finding time.

## How the System Works

When **RUN** is pressed, the configured number of drones are spawned at safe positions and begin operating autonomously. Each drone senses its local surroundings and uses the discovered information to update the swarm's belief grid, gradually building a representation of the initially unknown environment.

Using this knowledge, the system identifies unexplored boundaries and assigns exploration goals to the drones. Each drone moves towards its assigned goal while local swarm behaviours coordinate its movement, communication mechanisms support information sharing and connectivity, and avoidance behaviours respond to discovered obstacles and restricted areas.

This process continuously repeats as the swarm explores the environment. As new areas are discovered, the belief grid is updated, new exploration opportunities become available, and goals are reassigned when required. If a target is detected, an available drone is assigned to respond to it before normal exploration continues. Throughout the simulation, performance metrics monitor the swarm's exploration and target-finding performance.

The following flowchart shows the overall process and the main algorithm or mechanism responsible for each stage.

### Overall Simulation Flow

```text
┌──────────────────────────────────────────────┐
│                  PRESS RUN                   │
│                                              │
│ Initialise simulation and spawn the          │
│ configured number of drones safely           │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│             ENVIRONMENT SENSING              │
│                                              │
│ Mechanism: Radius-Based Local Sensing        │
│                                              │
│ Each drone observes the environment          │
│ within its configured sensing radius.        │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│              UPDATE BELIEF GRID              │
│                                              │
│ Mechanism: Discretised Belief-Grid Mapping  │
│                                              │
│ Discovered cells are classified as:          │
│ EXPLORED / OBSTACLE / RESTRICTED / TARGET    │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
               ┌──────────────────┐
               │ Target Detected? │
               └───────┬──────────┘
                   No  │  Yes
              ┌────────┘    └─────────────┐
              │                           ▼
              │              ┌────────────────────────┐
              │              │   TARGET ASSIGNMENT    │
              │              │                        │
              │              │ Method: Nearest        │
              │              │ Available Drone        │
              │              └───────────┬────────────┘
              │                          │
              └──────────────┬───────────┘
                             │
                             ▼
┌──────────────────────────────────────────────┐
│           COMMUNICATION & CONNECTIVITY       │
│                                              │
│ Mechanisms:                                 │
│ • Range-Based Drone Communication            │
│ • Communication Graph                       │
│ • Connected-Component Analysis              │
│                                              │
│ Drones exchange known assignments and the    │
│ system evaluates direct and multi-hop        │
│ connectivity across the swarm.               │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│              FRONTIER DETECTION              │
│                                              │
│ Algorithm: Frontier-Based Exploration        │
│                                              │
│ Finds boundaries between explored and        │
│ still-unknown areas of the belief grid.      │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│           EXPLORATION GOAL SELECTION         │
│                                              │
│ Method: Cost-Based Frontier Assignment       │
│                                              │
│ Candidate goals consider:                    │
│ • Travel distance                            │
│ • Frontier spacing                           │
│ • Communication cost                         │
│ • Predicted connectivity cost                │
│                                              │
│ Suitable individual goals are assigned       │
│ to drones that require a new destination.    │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│                SWARM MOVEMENT                │
│                                              │
│ Algorithm: Boids-Inspired Swarm Behaviour    │
│                                              │
│ • Separation                                 │
│ • Alignment                                  │
│ • Cohesion                                   │
│ • Goal Attraction                            │
│                                              │
│ Each drone moves towards its individual      │
│ goal while coordinating with neighbours.     │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│               HAZARD AVOIDANCE               │
│                                              │
│ Mechanism: Local Reactive Avoidance          │
│                                              │
│ • Obstacle Avoidance                         │
│ • Restricted-Area Avoidance                  │
│                                              │
│ Movement is corrected when discovered        │
│ hazards are nearby.                          │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│                APPLY MOVEMENT                │
│                                              │
│ Update drone velocity and position and       │
│ handle environment boundaries.               │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────┐
│           UPDATE PERFORMANCE METRICS         │
│                                              │
│ • Current Map Coverage                       │
│ • Map Coverage Time                          │
│ • Target Find Time                           │
└──────────────────────┬───────────────────────┘
                       │
                       ▼
               ┌─────────────────┐
               │ Target Reached? │
               └───────┬─────────┘
                   No  │  Yes
              ┌────────┘   └──────────────┐
              │                           ▼
              │              ┌────────────────────────┐
              │              │   TARGET COMPLETED     │
              │              │                        │
              │              │ Mark target as found   │
              │              │ and temporarily pause  │
              │              │ the swarm.             │
              │              └───────────┬────────────┘
              │                          │
              └──────────────┬───────────┘
                             │
                             ▼
┌──────────────────────────────────────────────┐
│             CONTINUE EXPLORATION             │
│                                              │
│ Completed or invalid goals are released,     │
│ new goals are assigned when required, and    │
│ the autonomous exploration cycle repeats.    │
└──────────────────────┬───────────────────────┘
                       │
                       └──────────────► REPEAT
```

> **Note:** The flowchart represents the logical operation of the system rather than completely separate sequential stages. Sensing, belief-grid updates, communication, movement, avoidance, target handling, and performance monitoring occur repeatedly while the simulation is running. More computationally expensive exploration and connectivity planning is performed less frequently to maintain smooth simulation performance.

### Environment & Belief Grid

The exploration system is based on two separate representations of the environment: the **ground-truth world** and the **belief grid**. This separation ensures that the drones make autonomous decisions using only the information they have discovered rather than having access to the complete map.

#### Ground-Truth Environment

The `World` represents the actual simulated environment. It stores the environment boundaries and the real locations of obstacles, restricted areas, and targets. These objects exist from the beginning of the simulation, but their locations are not automatically known by the swarm.

This allows the environment to act as the hidden ground truth against which the drones perform exploration.

#### Local Sensing

Each drone discovers the environment using **radius-based local sensing**. A drone can only observe locations that fall within its configured sensing radius around its current position.

As the drone moves, its sensing area moves with it. Newly observed areas are examined and the corresponding information is transferred from the ground-truth environment into the swarm's belief grid.

Therefore, the drones gradually discover the environment through exploration rather than receiving the complete map at the beginning.

#### Discretised Belief Grid

The `BeliefGrid` represents the environment as a collection of discrete grid cells. Each cell stores the swarm's current knowledge of that part of the environment.

A cell can contain one of five states:

| State | Meaning |
|---|---|
| `UNKNOWN` | The area has not yet been discovered. |
| `EXPLORED` | The area has been sensed and is known to be free. |
| `OBSTACLE` | An obstacle has been detected. |
| `RESTRICTED` | A restricted area has been detected. |
| `TARGET` | A target has been detected. |

At the beginning of exploration, most cells are `UNKNOWN`. As drones move and sense their surroundings, the belief grid is progressively updated.

The relationship between the two representations can therefore be summarised as:

```text
Ground-Truth World
        │
        │ Local Radius-Based Sensing
        ▼
     Drone
        │
        │ Discovered Information
        ▼
   Belief Grid
        │
        ├── UNKNOWN
        ├── EXPLORED
        ├── OBSTACLE
        ├── RESTRICTED
        └── TARGET
```

The belief grid then becomes the main source of environmental knowledge used by the autonomous exploration system. In particular, the boundary between `EXPLORED` and `UNKNOWN` cells is used to identify **frontiers**, which provide potential locations for the drones to explore next.

### Autonomous Frontier Exploration

Once the belief grid begins to contain explored areas, the swarm needs to decide where to move next. The simulator uses **frontier-based exploration** for this purpose. Instead of selecting random destinations, the system identifies useful locations at the boundary between areas the swarm already knows and areas that remain unexplored.

#### Frontier Detection

A **frontier** is an `EXPLORED` cell that is directly adjacent to at least one `UNKNOWN` cell. These cells represent the edge of the swarm's current knowledge and are useful exploration destinations because moving towards them is likely to reveal new parts of the environment.

Conceptually:

```text
UNKNOWN   UNKNOWN   UNKNOWN
UNKNOWN   FRONTIER  UNKNOWN
EXPLORED  EXPLORED  EXPLORED
```

The exploration system searches the belief grid for these boundaries and converts suitable frontier cells into world coordinates that can be used as potential drone destinations.

#### Frontier Goal Selection

Detecting a frontier does not automatically mean that a drone will move towards it. There may be many available frontiers at the same time, so the system evaluates candidate locations before assigning them.

The selection process considers several factors:

- **Travel Distance** – Frontiers closer to the drone are generally preferred because they require less travel.
- **Frontier Spacing** – Goals are kept apart to reduce the chance of multiple drones being assigned to nearly the same exploration location.
- **Communication Cost** – Candidate locations are evaluated according to their relationship with nearby drones and the communication radius.
- **Predicted Connectivity Cost** – The system estimates how selecting a candidate goal could affect the future communication structure of the swarm.

These factors are combined into a cost-based decision:

```text
Candidate Frontier
       │
       ├── Travel Distance
       ├── Frontier Spacing
       ├── Communication Cost
       └── Predicted Connectivity Cost
                │
                ▼
          Evaluate Candidate
                │
                ▼
        Select Suitable Goal
```

The system therefore attempts to balance **efficient exploration** with **swarm distribution and communication connectivity**, rather than simply sending every drone towards the closest unknown area.

#### Individual Goal Assignment

Exploration goals are assigned to individual drones rather than giving the entire swarm one shared destination. This allows different drones to investigate different parts of the environment simultaneously.

Each assigned destination is stored as the drone's `exploration_goal`. The goal then provides the desired exploration direction, while the drone's actual movement is still influenced by swarm coordination and avoidance behaviours.

This means:

```text
Drone 1 ──► Frontier A
Drone 2 ──► Frontier B
Drone 3 ──► Frontier C
Drone 4 ──► Frontier D
```

The drones can therefore spread across useful exploration areas while still behaving as members of the same swarm.

#### Goal Validation and Reassignment

Exploration goals are not permanent. As the swarm moves and the belief grid changes, an assigned frontier may be reached or may no longer represent a useful exploration boundary.

Existing goals are therefore checked during exploration. When a drone reaches its goal, or when the goal is no longer valid, it is released so that another suitable frontier can be assigned.

This creates a continuous autonomous exploration cycle:

```text
Detect Frontiers
       ↓
Evaluate Candidates
       ↓
Assign Individual Goals
       ↓
Explore
       ↓
Update Belief Grid
       ↓
Validate Existing Goals
       ↓
Goal Completed / Invalid?
       │
       ├── No ──► Continue towards goal
       │
       └── Yes ─► Release goal
                         │
                         ▼
                  Find New Frontier
                         │
                         └────► Repeat
```

Frontier exploration determines **where a drone should go**, but it does not directly control every movement the drone makes to get there. The assigned goal is passed to the movement system, where **Goal Attraction, Separation, Alignment, Cohesion, and avoidance behaviours** determine how the drone moves towards that destination.

### Swarm Behaviour

Once a drone has been assigned an exploration or target goal, the movement system determines **how the drone should move towards that destination while remaining part of the swarm**. The simulator uses a **Boids-inspired approach**, where coordinated swarm behaviour emerges from local interactions between neighbouring drones rather than from a single controller directly specifying the movement of the entire swarm.

Each drone calculates its movement independently using information about nearby drones and its current goal. The main swarm behaviours are **Separation, Alignment, and Cohesion**, which are combined with **Goal Attraction** to produce the drone's desired movement.

#### Neighbour Detection

Each drone has a configurable **neighbour radius**. Other drones located within this radius are treated as local neighbours and can influence its swarm behaviour.

```text
                 Drone
                   ●
             . . . . . . .
          .                 .
        .   ●           ●     .
       .       Neighbours      .
        .                   .
          .               .
             . . . . . .
          Neighbour Radius
```

This creates a **local swarm system**. A drone does not need to consider every other drone in the environment; it reacts primarily to drones within its local neighbourhood.

#### Separation

**Separation** prevents drones from becoming too close to each other. When neighbouring drones enter the configured separation distance, the drone calculates a steering influence away from them.

```text
Drone A  ◄──────►  Drone B
        Separation
```

This helps reduce crowding and prevents multiple drones from occupying almost the same position while exploring.

#### Alignment

**Alignment** encourages neighbouring drones to move in similar directions. A drone observes the movement direction of nearby drones and calculates an influence towards their general direction of travel.

```text
Drone A  ─────►
Drone B  ────►
Drone C  ─────►
```

This helps produce coordinated movement instead of each drone behaving completely independently.

#### Cohesion

**Cohesion** encourages a drone to remain associated with its local group. The drone calculates the approximate centre of its nearby neighbours and receives a movement influence towards that local centre.

```text
● Drone                         Drone ●
        \                       /
         \                     /
          \                   /
           ───► Local Centre ◄───
                   ●
```

Cohesion does **not** mean that every drone moves towards one global centre or that the entire swarm must remain in a single tight cluster. The local centre is continuously recalculated from the neighbours currently within each drone's neighbour radius.

#### Goal Attraction

While the three Boids behaviours coordinate local swarm movement, each drone must still progress towards its assigned exploration or target destination. **Goal Attraction** provides a steering influence towards the drone's current goal.

```text
Swarm Behaviour
      +
Goal Attraction
      ↓
Desired Movement Direction
```

This creates a balance between two requirements:

- moving towards useful exploration or target locations; and
- maintaining coordinated behaviour with nearby drones.

Without Goal Attraction, the drones could maintain local swarm behaviour without making useful progress towards their assigned destinations. Conversely, relying only on Goal Attraction would cause drones to travel towards their goals without considering the movement of nearby swarm members.

#### Combined Movement Behaviour

The movement direction is therefore influenced by several behaviours rather than being determined by a single rule:

```text
                    Separation
                         │
                    Alignment
                         │
                     Cohesion
                         │
                  Goal Attraction
                         │
                         ▼
              Combined Steering Influence
                         │
                         ▼
               Drone Movement Direction
```

Each behaviour has an adjustable weight that controls how strongly it influences the resulting movement. This allows the balance between exploration and swarm coordination to be changed and evaluated during testing.

The resulting direction is not necessarily a straight line towards the assigned goal. A drone may adjust its direction because of neighbouring drones while continuing to make progress towards its destination.

Obstacle and restricted-area avoidance can then further modify this movement when a known hazard is nearby.

#### Neighbour Radius vs Communication Radius

The **neighbour radius** used for swarm behaviour is different from the **communication radius** used by the communication system.

- **Neighbour Radius** determines which nearby drones influence Separation, Alignment, and Cohesion.
- **Communication Radius** determines which drones can directly exchange information and form communication links.

A drone can therefore be within communication range without necessarily being close enough to influence another drone's local swarm behaviour.

This separation allows the drones to spread across the environment for exploration while still maintaining communication links across a wider area.

### Communication & Connectivity

Communication allows the drones to exchange information and helps the swarm remain coordinated while individual drones explore different parts of the environment. The communication system is separate from the neighbour-based swarm behaviour used for Separation, Alignment, and Cohesion.

#### Communication Radius

Each drone has a configurable **communication radius**. Two drones can communicate directly when the distance between them is within this range.

```text
Drone A ●────────────● Drone B
          Direct
       Communication
```

If the distance between two drones exceeds the communication radius, they cannot communicate directly.

However, direct communication between every pair of drones is not required. Information can propagate through intermediate drones, allowing the swarm to form a **multi-hop communication network**.

For example:

```text
D1 ───── D2 ───── D3 ───── D4
```

D1 and D4 may be outside each other's direct communication radius, but they can still belong to the same connected network through D2 and D3.

#### Information Sharing

When drones are within communication range, they exchange information about known exploration-goal assignments. This helps drones become aware of goals already being handled by other members of the swarm.

Known assignment information can also propagate through connected drones, allowing information to travel beyond a single direct communication link.

#### Communication Graph

The communication network is represented as a **graph**:

- Each drone represents a **node**.
- Each direct communication link represents an **edge**.
- Groups of connected drones form **connected components**.

```text
Connected Swarm

D1 ───── D2 ───── D3
          │
          │
          D4


Disconnected Drone

D1 ───── D2 ───── D3              D4
```

Connected-component analysis allows the simulation to determine whether the swarm currently forms one communication network or has separated into multiple groups.

#### Connectivity-Aware Exploration

Communication is also considered when exploration goals are evaluated. A candidate frontier is not selected only because it is close to a drone. The exploration system also considers the potential effect of that destination on communication connectivity.

The system predicts the communication structure using the possible future positions of the drones and applies additional cost when a candidate goal is likely to produce a more disconnected swarm.

This allows the exploration system to balance two competing objectives:

```text
Explore New Areas
       ↕
Maintain Useful Connectivity
```

Communication radius therefore does **not** define the maximum distance of an exploration goal. A goal may be farther away than the communication radius because communication radius describes **drone-to-drone connectivity**, not **drone-to-goal distance**.

#### Connectivity Repair

If the communication graph becomes separated, the system identifies the largest connected component as the main connected part of the swarm. Drones outside this component are treated as disconnected.

For disconnected drones that are available for normal exploration, the system can evaluate frontier locations that encourage movement towards the main connected component while still contributing to exploration.

This provides a reactive mechanism for improving connectivity without requiring every drone to remain permanently inside direct communication range.


### Obstacle & Restricted-Area Avoidance

Exploration goals and swarm coordination determine where the drones generally want to move, but the drones must also react safely to hazards within the environment.

The simulator therefore provides separate avoidance behaviour for **obstacles** and **restricted areas**.

#### Obstacle Avoidance

Obstacles exist in the ground-truth environment but only become available to the autonomous movement system after they have been discovered through sensing and represented in the belief grid.

When a known obstacle is sufficiently close to a drone, the drone calculates an avoidance influence that steers its movement away from the hazardous area.

```text
Desired Movement
      ─────────────►

              █████
Drone ●       █████  Obstacle
      ↘       █████
       ↘
        ─────► Corrected Movement
```

This allows the drone to modify its path while continuing to make progress towards its assigned goal.

#### Restricted-Area Avoidance

Restricted areas are handled separately from physical obstacles. When a restricted area has been discovered and is close enough to affect the drone, a steering influence pushes the drone away from that region.

```text
Drone Movement
      │
      ▼
Restricted Area Detected Nearby?
      │
   Yes│
      ▼
Calculate Direction Away
      │
      ▼
Modify Movement
```

Obstacle and restricted-area avoidance have separate configurable weights. This allows their influence on movement to be adjusted independently during testing.

Avoidance operates alongside the other movement behaviours. Therefore, the final movement of a drone can be influenced by:

```text
Goal Attraction
      +
Separation
      +
Alignment
      +
Cohesion
      +
Obstacle Avoidance
      +
Restricted-Area Avoidance
      ↓
Final Movement Direction
```

This allows the drone to continue pursuing its autonomous exploration objective while reacting to hazards discovered during the mission.


### Target Detection & Response

Targets are part of the ground-truth environment but are initially unknown to the swarm. Their locations are not automatically provided to the drones when exploration begins.

A target becomes known only when it enters the sensing range of a drone.

#### Target Detection

When a drone senses a target, the target is recorded in the belief grid as `TARGET` and its detection state is updated.

```text
Unknown Target
      │
      │ Enters Sensing Radius
      ▼
Target Detected
      │
      ▼
Update Belief Grid
```

The target is then available to the target-management system.

#### Drone Assignment

Once a target has been detected, the system selects the **nearest available drone** and assigns the target to that drone.

The assigned drone receives a `target_goal`, which temporarily takes priority over its normal `exploration_goal`.

```text
             Detected Target
                    ●
                   /|\
                  / | \
                 /  |  \
               D1   D2   D3
                    ▲
                    │
             Nearest Available
                  Drone
```

This allows the rest of the swarm to continue its exploration behaviour while an appropriate drone responds to the detected target.

#### Reaching the Target

The assigned drone moves towards the target using the same movement system while continuing to respond to swarm and avoidance behaviours.

When the drone reaches the target within the required tolerance, the target is marked as **found**.

The swarm then temporarily pauses for the configured target-pause period before normal autonomous exploration continues.

```text
Target Detected
      ↓
Select Nearest Available Drone
      ↓
Assign Target Goal
      ↓
Drone Moves Towards Target
      ↓
Target Reached
      ↓
Mark Target as Found
      ↓
Temporarily Pause Swarm
      ↓
Resume Exploration
```

This target-handling process allows target discovery and response to occur as part of the autonomous exploration mission rather than requiring the target location to be known in advance.


## GUI & Controls

The simulator provides a full-screen **Pygame graphical interface** for configuring the environment, controlling the simulation, adjusting swarm parameters, and observing autonomous behaviour.

The interface separates commonly used simulation controls from more detailed development and diagnostic controls through **Normal Mode** and **Developer Mode**.

### Normal Mode

Normal Mode provides the main controls required to create and run a simulation without displaying unnecessary technical information.

It provides access to functions such as:

- Starting, pausing, and resetting the simulation.
- Selecting the number of drones.
- Editing the simulation environment.
- Placing obstacles, restricted areas, and targets.
- Adjusting the map-editing brush.
- Saving and loading environments.
- Viewing simulation status and performance metrics.

This provides a simplified interface for running and observing normal swarm simulations.

### Developer Mode

Developer Mode exposes additional parameters and diagnostic visualisations used to analyse and tune swarm behaviour.

The additional controls include important parameters such as:

- Sensing radius.
- Neighbour radius.
- Communication radius.
- Separation weight.
- Alignment weight.
- Cohesion weight.
- Goal Attraction weight.
- Obstacle Avoidance weight.
- Restricted-Area Avoidance weight.

Developer Mode also provides optional visualisations for internal swarm behaviour, including:

- Sensing-radius indicators.
- Neighbour-radius indicators.
- Communication-radius indicators.
- Neighbour links.
- Communication links.
- Exploration/frontier goals.

These visualisations make it easier to observe how changes to the swarm parameters affect the internal operation of the simulation.

### Map Editor & Map Management

The environment can be created directly through the graphical interface without modifying the source code.

The map editor supports:

- Painting obstacles.
- Painting restricted areas.
- Erasing previously painted areas.
- Changing the brush size.
- Placing targets.
- Saving environments.
- Loading previously created environments.

Saved maps are stored using **JSON** and contain the environment dimensions together with obstacle, restricted-area, and target information.

Simulation-specific state such as drone positions, current exploration goals, belief-grid exploration progress, and timers is not stored as part of the map. This allows a saved environment to be reused for new simulation runs.


## Algorithm Parameters

The simulator exposes several parameters that control how drones sense, communicate, coordinate, and move through the environment. These parameters can be adjusted to evaluate how different configurations affect swarm performance.

| Parameter | Purpose |
|---|---|
| **Sensing Radius** | Determines how far around itself a drone can observe and discover the environment. |
| **Neighbour Radius** | Determines which nearby drones influence local Separation, Alignment, and Cohesion behaviour. |
| **Communication Radius** | Determines the maximum range for direct communication between drones. |
| **Separation Weight** | Controls the influence of movement away from drones that become too close. |
| **Alignment Weight** | Controls how strongly a drone attempts to align its movement with nearby drones. |
| **Cohesion Weight** | Controls how strongly a drone is influenced towards the local centre of neighbouring drones. |
| **Goal Attraction Weight** | Controls the influence of the drone's assigned exploration or target goal on its movement. |
| **Obstacle Avoidance Weight** | Controls the influence of avoidance when a known obstacle is nearby. |
| **Restricted-Area Avoidance Weight** | Controls the influence of avoidance when a known restricted area is nearby. |

The movement weights do not represent identical physical forces. Each controls the influence of a different behaviour within the movement system.

Adjusting these parameters allows different swarm configurations to be tested and compared without redesigning the underlying algorithms.


## Performance Metrics

Performance metrics are recorded during the simulation to evaluate how effectively and efficiently the swarm completes its exploration and target-finding objectives.

### Map Coverage

Map coverage represents the percentage of the belief grid that is no longer unknown.

It is calculated using:

```text
               Known Belief-Grid Cells
Map Coverage = ───────────────────────── × 100
                Total Belief-Grid Cells
```

Cells classified as explored, obstacle, restricted, or target are considered known because the swarm has discovered information about those locations.

### Map Coverage Time

Map Coverage Time records how long the swarm takes to reach the configured map-coverage threshold.

The current simulation uses a **95% coverage threshold**. Once this threshold is reached, the recorded coverage time is frozen.

This provides a measurable way of comparing how efficiently different swarm configurations explore the environment.

### Target Find Time

Target Find Time measures how long the swarm takes to **first detect a target**.

The timer begins when the simulation starts and freezes when the first target is detected by any drone.

The metric therefore measures the swarm's target-discovery performance rather than the additional time required for the assigned drone to physically reach the target.


## Project Structure

The project is separated into GUI, simulation, and model components so that different parts of the system can be developed and maintained independently.

```text
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

| File / Directory | Responsibility |
|---|---|
| `main.py` | Application entry point that starts the simulator. |
| `config.json` | Stores configurable project and simulation parameters. |
| `config_loader.py` | Loads configuration values from `config.json`. |
| `gui/app.py` | Main GUI controller and application loop. |
| `gui/components.py` | Reusable GUI controls and components. |
| `gui/map_editor.py` | Environment editing and map save/load functionality. |
| `gui/renderer.py` | Renders the environment, drones, diagnostics, controls, and metrics. |
| `gui/settings.py` | Stores GUI-related settings and constants. |
| `models/world.py` | Represents the ground-truth simulation environment. |
| `models/belief.py` | Implements the discretised belief grid and frontier detection. |
| `models/drone.py` | Contains individual drone state and movement behaviour. |
| `simulation/simulation.py` | Coordinates the overall simulation update cycle. |
| `simulation/exploration.py` | Handles frontier evaluation and exploration-goal assignment. |
| `simulation/communication.py` | Handles communication-network and connectivity behaviour. |
| `simulation/target_manager.py` | Handles target detection, assignment, and completion behaviour. |
| `simulation/metrics.py` | Calculates and records simulation performance metrics. |
| `maps/` | Directory for storing reusable JSON simulation maps. |


## Installation & Running

The simulator requires **Python**, **Pygame**, and **NumPy**.

### 1. Clone the Repository

Clone the project and move into the project directory:

```bash
git clone <repository-url>
cd Capstone-Project---Swarm-Algorithm-
```

### 2. Install Dependencies

Install the required Python packages:

```bash
pip install numpy pygame
```

### 3. Run the Simulator

Run the application from the project directory:

```bash
python main.py
```

The simulator will open in full-screen mode.

Press `Esc` to exit the application.


## Current Development & Future Work

The current simulator provides the main foundation required for autonomous multi-drone exploration, including belief-grid mapping, frontier exploration, swarm coordination, communication-aware goal selection, connectivity handling, hazard avoidance, target response, environment editing, and performance measurement.

Further development will focus on improving and evaluating the existing system rather than replacing the core architecture. Areas for continued development include:

- Improving frontier distribution to reduce unnecessary travel between distant areas.
- Further improving communication-connectivity and reconnection behaviour.
- Refining and expanding performance metrics for experimental evaluation.
- Improving parameter optimisation for different swarm sizes and environments.
- Profiling and improving performance with larger numbers of drones.
- Evaluating the effect of different swarm and communication parameters.
- Improving path planning around more complex obstacle arrangements where required.
- Moving remaining fixed simulation parameters into the central configuration system.
- Conducting systematic experiments to evaluate exploration efficiency, target-finding performance, connectivity, and movement stability.
