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

When **RUN** is pressed, the simulation begins by creating the configured number of drones at safe positions within the environment. Each drone starts with only limited knowledge of the world, while the ground-truth environment already contains any obstacles, restricted areas, and targets created by the user.

From this point, the simulation operates autonomously. Each drone continuously senses the area within its sensing radius, and the information it discovers is recorded in the belief grid. As the drones explore, previously unknown cells are gradually identified as explored areas, obstacles, restricted areas, or targets.

Using this continuously updated knowledge, the exploration system identifies **frontiers**, which represent the boundaries between explored and still-unknown areas. Suitable frontier goals are then assigned to drones that require a new exploration destination. Goal selection considers factors such as the distance to the frontier, existing goal assignments, communication conditions, and the predicted effect of the goal on swarm connectivity.

Each drone moves towards its individual goal while its movement is simultaneously influenced by local swarm behaviours. **Separation** helps prevent drones from becoming too close to each other, **Alignment** encourages neighbouring drones to move in similar directions, and **Cohesion** encourages local group coordination. **Goal Attraction** guides each drone towards its assigned exploration or target goal. At the same time, **Obstacle Avoidance** and **Restricted-Area Avoidance** influence movement when discovered hazards are nearby.

Communication also occurs throughout exploration. Drones within communication range exchange known exploration assignments, while the communication system evaluates the network formed between the drones. Communication can occur through direct links or through other drones using multi-hop connectivity. If part of the swarm becomes disconnected, the connectivity logic can influence exploration and attempt to guide disconnected drones towards locations that improve reconnection.

As the drones continue moving and sensing the environment, the belief grid continues to expand. New frontiers become available while existing exploration goals may be completed or become invalid. The exploration planner therefore continues evaluating the current state of the environment and assigns new goals when required.

If a target enters a drone's sensing range, the target is detected and added to the swarm's known environment. An available drone is then assigned to the target, and reaching the target temporarily takes priority over its normal exploration goal. Once the assigned drone reaches the target, the target is marked as found and the entire swarm temporarily pauses before autonomous exploration resumes.

Throughout the simulation, performance metrics monitor the progress of the swarm. These include the percentage of the environment that has been mapped, the time required to reach the configured map-coverage threshold, and the time at which the first target is detected.

This continuous cycle of **sensing, mapping, communication, planning, coordinated movement, avoidance, target handling, and performance measurement** continues until the simulation is paused, reset, or terminated by the user.

> **Note:** These processes do not operate as completely separate stages. Sensing, movement, avoidance, communication, target handling, and other behaviours are repeatedly performed throughout the simulation. More computationally expensive exploration and connectivity planning is performed at a lower frequency to maintain simulation performance.

### Overall Simulation Flow

```text
                         ┌─────────────────────┐
                         │      Press RUN      │
                         └──────────┬──────────┘
                                    │
                                    ▼
                     ┌──────────────────────────┐
                     │ Spawn Drones at Safe     │
                     │ Starting Positions       │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Sense Local Environment  │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Update Shared Belief Grid│
                     │                          │
                     │ Unknown → Explored       │
                     │         → Obstacle       │
                     │         → Restricted     │
                     │         → Target         │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Target Detected?         │
                     └───────┬──────────┬───────┘
                             │ No       │ Yes
                             │          ▼
                             │   ┌────────────────────┐
                             │   │ Assign Available   │
                             │   │ Drone to Target    │
                             │   └─────────┬──────────┘
                             │             │
                             ▼             ▼
                ┌──────────────────────────────────┐
                │ Communication & Connectivity     │
                │                                  │
                │ • Exchange known assignments     │
                │ • Build communication network    │
                │ • Evaluate connectivity          │
                └────────────────┬─────────────────┘
                                 │
                                 ▼
                ┌──────────────────────────────────┐
                │ Exploration Planning             │
                │                                  │
                │ • Detect frontiers               │
                │ • Validate existing goals        │
                │ • Evaluate candidate frontiers   │
                │ • Assign individual goals        │
                └────────────────┬─────────────────┘
                                 │
                                 ▼
                ┌──────────────────────────────────┐
                │ Calculate Drone Movement         │
                │                                  │
                │ • Goal Attraction                │
                │ • Separation                     │
                │ • Alignment                      │
                │ • Cohesion                       │
                │ • Obstacle Avoidance             │
                │ • Restricted-Area Avoidance      │
                └────────────────┬─────────────────┘
                                 │
                                 ▼
                     ┌──────────────────────────┐
                     │ Update Drone Positions   │
                     │ & Handle Boundaries      │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Update Performance       │
                     │ Metrics                  │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Target Reached?          │
                     └───────┬──────────┬───────┘
                             │ No       │ Yes
                             │          ▼
                             │   ┌────────────────────┐
                             │   │ Mark Target Found  │
                             │   │ & Pause Swarm      │
                             │   └─────────┬──────────┘
                             │             │
                             └──────┬──────┘
                                    │
                                    ▼
                     ┌──────────────────────────┐
                     │ Continue Autonomous      │
                     │ Exploration             │
                     └────────────┬─────────────┘
                                  │
                                  └──────────► Repeat
```

### Environment & Belief Grid


### Autonomous Frontier Exploration


### Swarm Behaviour


### Communication & Connectivity


### Obstacle & Restricted Area Avoidance


### Target Detection & Response


## GUI & Controls

### Normal Mode


### Developer Mode


### Map Editor


## Algorithm Parameters


## Performance Metrics


## Project Structure


## Installation & Running


## Current Development & Future Work