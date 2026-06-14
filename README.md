*This project has been created as part of the 42 curriculum by kimendon.*

# Fly-In — Drone Routing Simulation

## Description

**Fly-In** is a drone routing simulation engine written in Python. Given a network of connected zones and a fleet of drones, the system finds optimal paths from a start hub to an end hub and animates the step-by-step movement of every drone, minimizing the total number of simulation turns.

The project covers:
- Parsing and validating a custom map file format describing zones and connections.
- Building a graph from that map and transforming it into a residual flow network.
- Finding capacity-aware, cost-weighted paths using a **Min-Cost Max-Flow** algorithm (Bellman-Ford based).
- Scheduling drones across discovered paths to minimize the makespan (last drone arrival turn).
- Running a turn-by-turn simulation respecting zone occupancy and connection capacity limits.
- Rendering the result as an interactive animated graph (Plotly) and structured terminal output.

---

## Instructions

### Requirements

- Python **3.10** or later
- A virtual environment is recommended

### Installation

```bash
make install
```

This creates a `.venv` virtual environment and installs all dependencies listed in `requirements.txt` (`pydantic`, `plotly`, `flake8`, `mypy`).

### Running the simulation

```bash
make run
```

The program will prompt you to choose a map category (`easy`, `medium`, `hard`, `challenger`) and then the specific map file. After path-finding completes, it prints the turn-by-turn terminal output and opens an interactive Plotly animation in the browser.

You can also pass a map directly in code — see the commented-out `__main__` blocks in `simulation_engine.py`.

### Debug mode

```bash
make debug
```

Runs `main.py` under Python's built-in `pdb` debugger.

### Linting and type checking

```bash
make lint          # flake8 + mypy (standard flags)
make lint-strict   # flake8 + mypy --strict
```

### Cleaning build artifacts

```bash
make clean
```

Removes `__pycache__` and `.mypy_cache`.

---

## Algorithm Choices and Implementation Strategy

The solution is built on a pipeline of five stages, each encapsulated in its own class.

### 1. Parsing — `NetworkParser` / `Network` (`input_parser.py`, `models.py`)

The map file is read line by line. Each line is split on `": "` to extract a prefix (`nb_drones`, `start_hub`, `end_hub`, `hub`, `connection`) and a value. Zone and connection metadata inside `[...]` is parsed separately. All values are fed into **Pydantic** models (`DronesN`, `Node`, `Connection`, `Network`) which enforce types, ranges, and cross-field constraints (unique zone names, valid connections, no duplicate edges). Any violation raises a typed `NetworkFileError` subclass with the source file line number.

### 2. Graph — `Graph` (`graph.py`)

A lightweight wrapper around the parsed `Node` and `Connection` lists that provides neighbour lookups (`get_node_edges`), edge capacity queries (`get_edge`), and unified access to all nodes including start and end.

### 3. Flow graph and pathfinding — `FlowGraph` / `MinCostMaxFlowAlgorithm` (`min_cost_max_flow.py`)

To respect **zone capacity** (`max_drones`) as a vertex capacity, each node is **split** into an `_in` and `_out` sub-node connected by an inner edge whose capacity equals `max_drones` and whose cost encodes the zone type:

| Zone type   | Inner edge cost |
|-------------|-----------------|
| `priority`  | -2              |
| `normal`    | 1               |
| `restricted`| 10              |
| `blocked`   | not added       |

Physical connections between zones become edges between `_out` of one node and `_in` of another, with capacity equal to `max_link_capacity` and cost `1`. Connections between two priority zones use an external edge cost of **2** (instead of 1) to prevent negative cycles while preserving the cost advantage of priority paths.

**Bellman-Ford** finds the shortest (cheapest) augmenting path in the residual graph, `augment()` pushes flow along it, and the loop repeats until no augmenting path exists. `decompose_paths()` then reads the final positive-flow edges and reconstructs source-to-sink paths. `collapse_paths()` strips the `_in`/`_out` suffixes to recover original node names.

This approach gives us a set of candidate paths that already respect all capacity constraints and prefer `priority` zones.

**Complexity:** O(V · E) per Bellman-Ford iteration × number of augmenting paths found. In practice the number of paths is bounded by the network's maximum flow, which is small for the provided maps.

### 4. Drone scheduling — `RoutePlanner` (`route_planner.py`)

Given the candidate paths, `RoutePlanner.schedule_drones()` tries every subset of paths (by size, from 1 upward) and for each subset greedily assigns drones one by one to the path that yields the earliest expected completion time:

```
completion_time = path_cost + ceil(drones_on_path / bottleneck_capacity) - 1
```

where `path_cost` sums zone-type movement costs (restricted = 2, others = 1) and `bottleneck_capacity` is the minimum of all zone and connection capacities along the path.

The subset that minimises `max(completion_time across all paths)` is chosen. Paths computed once by MCMF are not recomputed — O(2^P × D) where P is the number of distinct paths and D is the number of drones.

### 5. Simulation — `SimulationEngine` (`simulation_engine.py`)

Each turn, every drone that has not yet reached the end is asked for its next mode (`get_next_move()`). The method checks whether the next node has remaining occupancy (with `current_occupancy` a live capacity ledger), handles the two-turn restricted zone transit (drone stays on the edge name for one turn, then enters the zone), and the drone waits if the next node is full (with a return of `None`). Drones moving out of a node free up capacity before drones moving in consume it, matching the spec's same-turn freeing rule.

After each turn, the structured output (`D<id>-<zone>`) is printed and the Plotly frame is recorded.

---

## Visual Representation

Two complementary outputs are provided:

### Terminal output (mandatory format)

After path-finding, each simulation turn is printed as a space-separated list of drone movements:

```
D1-roof1 D2-corridorA
D1-roof2 D2-tunnelB
D1-goal D2-goal
```

Drones that stay in place or have already reached the end are omitted. This is the canonical scored output.

### Optional secondary metrics

After the simulation, `print_secondary_metrics()` displays:
- Drones moved per turn (throughput per step).
- Total turns taken per drone.
- Average turns per drone (ceiling-rounded).

### Interactive Plotly animation (`renderer.py`)

After the terminal output, an interactive HTML figure opens in the browser showing:
- **Grey lines** connecting nodes as edges.
- **Coloured circles** for each hub, using the `color` metadata from the map file.
- **Black triangles** for drones, with IDs as labels.
- A **Play / Pause** button that animates drone movement turn by turn (2.5 s per frame, 0.75 s transition).
- When multiple drones share a node, they are spread in a small orbit around the node centre to remain individually visible.

The animation makes it easy to visually verify that capacity rules are respected, identify bottlenecks, and understand the routing strategy chosen by the algorithm.

---

## Project Structure

| File | Role |
|------|------|
| `main.py` | Entry point — prompts for map |
| `models.py` | Pydantic data models: `ZoneType`, `Color`, `DronesN`, `Node`, `Connection` |
| `errors.py` | Custom exception hierarchy (`NetworkFileError` and subclasses, `GraphError` and `FlowGraphError`) |
| `input_parser.py` | `NetworkParser` (file → `Network`) and `Network` (cross-field validation) |
| `graph.py` | `Graph` — adjacency and capacity queries |
| `min_cost_max_flow.py` | `FlowGraph`, `Edge`, `MinCostMaxFlowAlgorithm` path-finding algorithm|
| `route_planner.py` | `Drone`, `RoutePlanner`, `PathsFindingAlgorithm` protocol |
| `simulation_engine.py` | `SimulationEngine`, `OptionalMetrics` |
| `renderer.py` | `Renderer` — Plotly animated visualisation |
| `Makefile` | `install`, `run`, `debug`, `lint`, `lint-strict`, `clean` |
| `requirements.txt` | `pydantic`, `plotly`, `flake8`, `mypy` |


---

## Resources

### References

- [Graph Theory and Graphs in Python](https://python-course.eu/applications-python/graphs-python.php) — explanation of graph theory and how they can be represented in Python
- [Bellman-Ford algorithm](https://en.wikipedia.org/wiki/Bellman%E2%80%93Ford_algorithm) — shortest path in graphs with negative weights
- [Min-Cost Max-Flow](https://en.wikipedia.org/wiki/Minimum-cost_flow_problem) — flow optimization with costs
- [Flow decomposition](https://www.youtube.com/watch?v=JlzjTR4Y1FQ&t=10s) — recovering paths from a flow solution
- [Makespan minimization on parallel machines or greedy load-balancing algorithm](https://pdfs.semanticscholar.org/8eae/bd0a747a2e35964c3b1a04dafdedf63d949d.pdf) — the theoretical model behind drone-to-path assignment
- [Plotly Network Graphs in Python documentation](https://plotly.com/python/network-graphs/) — interactive graph visualization and animation

### AI Usage

- **Algorithm design discussion**: exploring pathfinding strategies (BFS, Dijkstra, MCMF) and understanding trade-offs between them for this specific problem.
- **Debugging**: explaining unexpected Pydantic validation errors and mypy warnings.
- **README drafting**: this file was drafted with AI assistance and reviewed for accuracy against the actual implementation.
