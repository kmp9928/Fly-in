from typing import List, Dict, Tuple
import copy
from math import ceil
from models import Node
from graph import Graph
from min_cost_max_flow import Path
from route_planner import Drone


class SimulationEngine:
    """Manages the tick-by-tick routing execution of the drone fleet over
    scheduled paths.

    Expands network path sequences to include structural edge positions,
    validates regional capacity ceilings, step-moves drone positions
    dynamically, logs metrics and manages structural turn-by-turn state
    histories.

    Attributes:
        paths: Explicit list of path steps with connection strings.
        drone_assignment: Fleet distribution maps matching drones to their
            initial chosen routes.
        turns: Maximum calculated turn cycles for the total simulation.
        graph: The source network map housing configuration nodes.
        current_occupancy: Live ledger tracking structural remaining capacity
            across nodes and paths.
        metrics: Internal analytics engine caching performance details.
    """
    paths: List[Path]
    drone_assignment: List[List[Drone]]
    turns: int
    graph: Graph
    current_occupancy: Dict[str, int]
    metrics: "OptionalMetrics"

    def __init__(
        self,
        paths: List[Path],
        drone_assignment: List[List[Drone]],
        turns: int,
        graph: Graph
    ) -> None:
        """Initializes tracking parameters, mapping explicit edge names and
        measuring route loads."""
        self.paths = SimulationEngine.insert_edges(paths)
        self.drone_assignment = drone_assignment
        self.turns = turns
        self.graph = graph
        self.current_occupancy = self.calculate_occupancy()
        self.metrics = SimulationEngine.OptionalMetrics()

    @staticmethod
    def insert_edges(paths: List[Path]) -> List[Path]:
        """Synthesizes path to include node connections.

        Transforms a raw sequence of points (e.g., `['A', 'B']`) into an
        explicit coordinate path containing transitional edge states
        (e.g., `['A', 'A-B', 'B']`).

        Args:
            paths: The list of raw node sequences.

        Returns:
            An expanded list of paths containing transitional edge elements.
        """
        path_with_edges: List[Path] = []
        for path in paths:
            new_path: Path = []
            for n in range(0, len(path)):
                new_path.append(path[n])
                if n < len(path) - 1:
                    new_path.append(path[n] + "-" + path[n + 1])
            path_with_edges.append(new_path)

        return path_with_edges

    def simulate_turns(self) -> Dict[int, Dict[str, List[Drone]]]:
        """Executes the step-by-step turn loop of the global drone simulation.

        Initializes initial fleet allocations at the network source node, steps
        forward drone positions sequentially each turn, handles restricted zone
        behavior and updates metrics logs.

        Returns:
            A chronological ledger history mapping turn IDs to consolidated
                node location structures.
        """
        simulation_history: Dict[int, Dict[str, List[Drone]]] = {}
        all_drones: List[Drone] = sorted(
            list(
                drone
                for drones_per_path in self.drone_assignment
                for drone in drones_per_path
            ),
            key=lambda drone: int(drone.id.strip("D"))
        )
        start: str = self.graph.get_start().name
        end: str = self.graph.get_end().name
        paths_state: List[Dict[str, List[Drone]]] = [
            {point: [] for point in path} for path in self.paths
        ]

        for index, path in enumerate(paths_state):
            path[start] = self.drone_assignment[index][:]

        for path in paths_state:
            for drone in path[start][:]:
                self.move_drones(path, start, start, drone)

        simulation_history[0] = SimulationEngine.consolidate_paths(
            copy.deepcopy(paths_state)
        )

        for turn in range(1, self.turns + 1):
            for drone in all_drones:
                if drone.current != end:
                    assert drone.current is not None
                    path_index = next(
                        path_i
                        for path_i, drone_list in enumerate(
                            self.drone_assignment
                        )
                        if any(d.id == drone.id for d in drone_list)
                    )
                    self.move_drones(
                        paths_state[path_index],
                        drone.current,
                        self.get_next_move(
                            self.paths[path_index],
                            drone.current
                        ),
                        drone
                    )
                    drone.turns += 1

            simulation_history[turn] = SimulationEngine.consolidate_paths(
                copy.deepcopy(paths_state)
            )
            self.print_simulation_output(simulation_history[turn])

            self.metrics.add_drones_moved_in_turn(
                [
                    drone
                    for path in paths_state
                    for drones in path.values()
                    for drone in drones
                ]
            )

            for path in paths_state:
                for drone_list in path.values():
                    for drone in drone_list:
                        SimulationEngine.switch_drone_status(drone, False)

        self.metrics.count_turns_per_drone(simulation_history[self.turns][end])

        return simulation_history

    def calculate_occupancy(self) -> Dict[str, int]:
        """Maps operational capacity ceilings to all active path features and
        intersections.

        Returns:
            A map registry linking node name symbols or synthetic edge
                connection strings to their numerical structural capacity caps.
        """
        return {
            point: (
                self.graph.get_node(point).max_drones
                if point in self.graph.get_all_nodes().keys()
                else self.graph.get_edge(
                    (point.split("-")[0], point.split("-")[1])
                )
            )
            for path in self.paths
            for point in path
        }

    def get_next_move(self, path: Path, current: str) -> str | None:
        """Calculates congestion bottlenecks to identify the next step target
        for a drone asset.

        Performs calculations evaluating normal and restricted zones, skips
        structural edge steps where safety ceilings permit, adjusts structural
        node capacity registries, and flags a stall condition if path options
        are congested.

        Args:
            path: The specific path tracking line assigned to the asset.
            current: The name identifier string where the target asset resides.

        Returns:
            The identifier of the next point or None if the drone must stall.
        """
        nodes: Dict[str, Node] = self.graph.get_all_nodes()
        current_index: int = path.index(current)

        if nodes.get(path[current_index + 1]) is None:
            if nodes[path[current_index + 2]].zone.value == "restricted":
                next_point = path[current_index + 1]
            else:
                if self.current_occupancy[path[current_index + 1]] == 0:
                    return None
                next_point = path[current_index + 2]
                if next_point != self.graph.get_end().name:
                    self.current_occupancy[path[current_index + 1]] -= 1
            if (
                current != self.graph.get_start().name and
                nodes[path[current_index]].zone.value != "restricted"
            ):
                self.current_occupancy[path[current_index - 1]] += 1
        else:
            next_point = path[current_index + 1]

        if self.current_occupancy[next_point] == 0:
            return None
        else:
            if current != self.graph.get_start().name:
                self.current_occupancy[current] += 1
            if next_point != self.graph.get_end().name:
                self.current_occupancy[next_point] -= 1
            return next_point

    def move_drones(
        self,
        state: Dict[str, List[Drone]],
        from_point: str,
        to_point: str | None,
        drone: Drone
    ) -> None:
        """Updates internal telemetry arrays, grid spaces and state keys for a
        moving drone.

        Args:
            state: Active state trackers isolated to a designated single-path
                pipeline array.
            from_point: Structural key identity string of the origin point.
            to_point: Structural key identity string of the target location.
            drone: The specific drone instance being modified.
        """
        if to_point is not None:
            state[to_point].append(drone)

            if "-" in to_point:
                drone.x, drone.y = self.calculate_edge_coordinate(to_point)
            else:
                drone.x = self.graph.get_node(to_point).x
                drone.y = self.graph.get_node(to_point).y

            drone.current = to_point

            if from_point != to_point:
                SimulationEngine.switch_drone_status(drone, True)

            current_index = state[from_point].index(drone)
            del state[from_point][current_index]

    def calculate_edge_coordinate(self, to_point: str) -> Tuple[float, float]:
        """Calculates the center-point spatial coordinates of an edge path.

        Splits an edge name string to track terminal points, mapping the
        midpoint position for visualization rendering.

        Args:
            to_point: The string of the compound edge connection (e.g., "A-B").

        Returns:
            A tuple representing calculated X and Y grid space coordinates.
        """
        from_node, to_node = to_point.split("-")
        from_x: int = self.graph.get_node(from_node).x
        to_x: int = self.graph.get_node(to_node).x
        from_y: int = self.graph.get_node(from_node).y
        to_y: int = self.graph.get_node(to_node).y

        x = ((to_x - from_x) / 2) + from_x
        y = ((to_y - from_y) / 2) + from_y

        return (x, y)

    @staticmethod
    def consolidate_paths(
        paths_state: List[Dict[str, List[Drone]]]
    ) -> Dict[str, List[Drone]]:
        """Combines single-path pipeline structures into a unified global
        network overview map."""
        consolidated_path: Dict[str, List[Drone]] = {}

        for path in paths_state:
            for point, drones in path.items():
                if consolidated_path.get(point) is None:
                    consolidated_path[point] = list(drones)
                else:
                    consolidated_path[point].extend(drones)

        return consolidated_path

    def print_simulation_output(self, output: Dict[str, List[Drone]]) -> None:
        """Prints a standardized logs line detailing asset movements during the
        current turn."""
        formatted_output: List[str] = []

        for point, drones in output.items():
            for drone in drones:
                if point == self.graph.get_start().name or not drone.moved:
                    continue
                formatted_output.append(drone.id + "-" + point)

        print((" ").join(sorted(formatted_output)))

    @staticmethod
    def switch_drone_status(drone: Drone, moved: bool) -> None:
        """Sets the action flag specifying whether a drone moved."""
        drone.moved = moved

    def print_secondary_metrics(self) -> None:
        """Aggregates and logs final deployment statistics to the terminal."""
        print("Number of drones moved per turn:")
        for n in range(len(self.metrics.get_drones_per_turn())):
            print(
                f"Turn {n + 1}: " +
                f"{self.metrics.get_drones_per_turn()[n]} drone(s)"
            )

        print("\nNumber of turns per drone:")
        for id, turns in self.metrics.get_turns_per_drone().items():
            print(f"Drone '{id}': {turns} turn(s)")

        av_turns = self.metrics.get_av_turns_per_drone()
        print(f"\nAverage number of turns per drone: {av_turns}")

    class OptionalMetrics:
        """Internal data collection container that records and compiles metrics
        for the fleet.

        Attributes:
            drones_per_turn: Chronological list tracking total assets moved on
                each step tick.
            turns_per_drone: Key dictionary recording cumulative travel times
                mapped to drone IDs.
        """
        drones_per_turn: List[int]
        turns_per_drone: Dict[str, int]

        def __init__(self) -> None:
            """Instantiates metric monitoring collections."""
            self.drones_per_turn = []
            self.turns_per_drone = {}

        def add_drones_moved_in_turn(self, drones: List[Drone]) -> None:
            """Calculates active step state completions to determine movement
            totals."""
            self.drones_per_turn.append(sum(drone.moved for drone in drones))

        def get_drones_per_turn(self) -> List[int]:
            """Retrieves the history of aggregate turn movements."""
            return self.drones_per_turn

        def count_turns_per_drone(self, drones: List[Drone]) -> None:
            """Maps total active travel windows for assets arriving at the
            destination node."""
            self.turns_per_drone.update(
                {drone.id: drone.turns for drone in drones}
            )

        def get_turns_per_drone(self) -> Dict[str, int]:
            """Retrieves alpha-sorted drone tracking tables pairing IDs to
            transit durations."""
            return {
                drone: turns
                for drone, turns in sorted(
                    self.turns_per_drone.items(),
                    key=lambda item: int(item[0].strip("D"))
                )
            }

        def get_av_turns_per_drone(self) -> int:
            """Computes the overall mathematical mean runtime duration across
            the active fleet."""
            return ceil(
                sum(turns for turns in self.turns_per_drone.values()) /
                len(self.turns_per_drone.keys())
            )
