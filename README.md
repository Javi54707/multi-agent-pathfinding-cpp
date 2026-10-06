# Multi-Agent Autonomous Routing & Synchronization System

## Overview
This project implements a C++ simulation of a multi-agent system designed to autonomously explore, navigate, and modify a procedurally generated grid environment. The core objective is to calculate and construct an optimal pipeline network across uneven terrain while strictly adhering to energy budgets, ecological impact limits, and physical gravity constraints (fluid dynamics).

The system features two distinct agents (Engineer and Technician) with asymmetric capabilities. They must synchronize their state machines to perform joint operations in a partially observable environment (Fog of War).

## Algorithmic Architecture & Key Features

### 1. Advanced Pathfinding & Optimization
*   **Cost-Aware A* Search:** Implemented A* using Chebyshev distance heuristics to optimize energy consumption over uneven terrain[cite: 15]. The heuristic is mathematically admissible, ensuring optimal path discovery[cite: 15].
*   **Multi-Objective A* with Pareto Pruning:** To route pipelines, the algorithm minimizes the number of segments while constrained by a strict "ecological budget"[cite: 19]. A custom dictionary tracks the minimum ecological impact per state $(f, c, \text{height modification})$, pruning branches that are suboptimal (Pareto-like pruning) to prevent memory saturation[cite: 19].
*   **Dynamic Gravity Constraints:** The state-space generation automatically evaluates terrain height alterations ($\pm 1$) to ensure water flows downwards or maintains elevation, dynamically factoring the ecological cost of terraforming (`RAISE`/`DIG` actions) into the node expansion[cite: 19].

### 2. Multi-Agent Synchronization (Master-Slave Architecture)
*   **State-Machine Coordination:** The agents utilize a Master-Slave synchronization model for joint operations[cite: 23]. The Engineer (Master) plans the global pipeline, navigates to nodes, conditions the terrain, and issues a `COME` signal[cite: 23]. The Technician (Slave) remains in an energy-saving `IDLE` state until called, calculates a dynamic route avoiding dynamic obstacles, and synchronizes orientation to execute the `INSTALL` action simultaneously[cite: 23].

### 3. Partial Observability & Dynamic Replanning
*   **Fog of War Navigation:** Agents operate with a limited visual cone (representing terrain type, height, and dynamic entities)[cite: 38]. 
*   **"GPS Gravity" Heuristic:** During blind exploration, the agents alter movement priorities to minimize the Manhattan distance toward known target coordinates, forcing progress through penalized terrain if necessary[cite: 28].
*   **Real-Time Collision Avoidance:** If an agent detects a previously hidden obstacle, the current search tree is aborted, the local map memory is updated, and the A* trajectory is dynamically recalculated[cite: 28].

## Technical Stack
*   **Language:** C++17
*   **Concepts:** Artificial Intelligence, Heuristic Search (A*, BFS), Multi-Agent Systems, Finite State Machines (FSM), Procedural Navigation.
*   **Dependencies:** `freeglut`, `openmpi`, `boost`, `cmake`[cite: 58].

## Build and Execution

The project supports both a GUI visualizer and a headless batch mode for performance testing and debugging.

### Compilation
A setup script is provided for Linux environments:
```bash
./install.sh
make
```

### Running the Simulation
To execute the simulation in headless mode (ideal for testing search efficiency):
```bash
./practica2SG -m ./mapas/mapa100.map -seed 0 -Tiempo 3000 -Ambiental 1719 -Energia 4581
```
*Parameters can be adjusted to test algorithm robustness under tighter energy (`-Energia`) or ecological (`-Ambiental`) constraints.*
