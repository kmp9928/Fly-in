from sys import maxsize
from typing import List, Tuple, Protocol
from itertools import combinations
from math import ceil
from dataclasses import dataclass
from graph import Graph
from min_cost_max_flow import Path


@dataclass
class Drone:
    """Tracks state data, coordinate positions and routing metrics for an
    independent drone.

    Attributes:
        id: Unique identifying string label for the drone instance.
        current: The name of the map node the drone is currently occupying.
        x: Horizontal coordinate positioning on the visual simulation grid.
        y: Vertical coordinate positioning on the visual simulation grid.
        moved: Flags whether this asset has been moved for the current turn.
        turns: Cumulative metric tracking the duration spent in transit.
    """
    id: str
    current: str | None = None
    x: float = 0
    y: float = 0
    moved: bool = False
    turns: int = 0


class PathsFindingAlgorithm(Protocol):
    """Structural blueprint interface defining valid network flow routing
    strategies."""

    def find_paths(self, graph: Graph) -> List[Path]:
        """Calculates a set of independent routing paths from network source
        to target sink.

        Args:
            graph: The concrete network graph instance to evaluate.

        Returns:
            A breakdown of route arrays mapped across valid nodes.
        """
        ...


class RoutePlanner:
    """Manages optimal distribution and scheduling configurations for
    multi-drone deployment.

    Evaluates path combinations to avoid node and edge bottleneck capacities
    and balances active fleet deployments globally across all discovered path
    subsets to minimize final deployment turns.

    Attributes:
        paths: Discovered routes derived from the pathfinder execution.
        graph: The topology model storing node attributes and fleet criteria.
    """
    paths: List[Path]
    graph: Graph

    def __init__(
        self, paths_finding_algorithm: PathsFindingAlgorithm, graph: Graph
    ) -> None:
        """Initializes structural path variants by executing the injected
        optimization algorithm."""
        self.paths = paths_finding_algorithm.find_paths(graph)
        self.graph = graph

    def schedule_drones(self) -> Tuple[List[Path], List[List[Drone]], int]:
        """Runs a combinatorial check across all path subsets to discover the
        fastest dispatch timeline.

        Iterates sequentially through mathematical path subset sizes,
        distributing the active fleet across candidate pathways and saving the
        allocation combination that delivers the minimum total completion
        timeframe.

        Returns:
            A tuple containing:
                - The optimal subset list of paths selected for deployment.
                - Nested lists pairing generated Drone tracking assets to their
                    specific path index.
                - The minimum runtime steps required to finalize the absolute
                    last drone arrival.
        """
        best_subset: List[Path] = []
        best_assignment: List[List[Drone]] = []
        min_turns: int = maxsize

        for size in range(1, len(self.paths) + 1):
            for subset in combinations(self.paths, size):
                turns, assignment = self.drone_assignment(list(subset))

                if turns < min_turns:
                    min_turns = turns
                    best_subset = list(subset)
                    best_assignment = assignment

        return (best_subset, best_assignment, min_turns)

    def drone_assignment(
        self, paths: List[Path]
    ) -> Tuple[int, List[List[Drone]]]:
        """Greedily delegates the fleet across targeted paths to minimize
        concurrent travel lag.

        Evaluates pipeline delays drone-by-drone to choose the best pathway
        candidate, updates path tracking capacities and captures global
        simulation completion times.

        Args:
            paths: The specific sub-collection of path to allocate assets into.

        Returns:
            A tuple containing:
                - The maximal turn duration needed for all routes to complete.
                - An indexed structure for drones and their chosen paths.
        """
        n_drones_per_path: List[int] = [0 for _ in paths]
        drones_per_path: List[List[Drone]] = [[] for _ in paths]
        turns_per_path: List[int] = [0 for _ in paths]

        for drone_n in range(1, self.graph.get_drones_n() + 1):
            best_path, turns = self.choose_best_path(
                paths, n_drones_per_path
            )
            drone = Drone(id="D" + str(drone_n))
            best_path_index = paths.index(best_path)
            drones_per_path[best_path_index].append(drone)
            n_drones_per_path[best_path_index] += 1
            turns_per_path[best_path_index] = turns

        return (max(turns_per_path), drones_per_path)

    def choose_best_path(
        self, paths: List[Path], drones_per_path: List[int]
    ) -> Tuple[Path, int]:
        """Identifies the optimal route candidate for the next drone allocation
        based on load.

        Calculates total weight factors by scoring local node types (such as
        zones flagged as restricted), checks for the lowest bottleneck
        threshold values along path edges and node intersections and predicts
        prospective completion windows.

        Args:
            paths: Current candidate paths available for assignment selection.
            drones_per_path: Concurrent allocation values mapping drone count
                totals to path indices.

        Returns:
            A tuple containing:
                - The optimal target Path for current assignment placement.
                - The predicted step timeframe required to finalize that route.
        """
        best_path: Path = []
        min_turns: int = maxsize

        for index, path in enumerate(paths):
            # print(f"path stats for path {path}")
            drones_in_path = drones_per_path[index] + 1

            total_turns = 0
            # print(f"path is {path[1:]}")
            for node in path:
                if node == self.graph.get_start().name:
                    continue
                if self.graph.get_node(node).zone.value == "restricted":
                    total_turns += 2
                else:
                    total_turns += 1

            # print(f"total_turns {total_turns}")
            bottleneck_capacity = min([
                self.graph.get_node(node).max_drones for node in path
            ])
            bottleneck_capacity_edges = []
            for index in range(0, len(path) - 1):
                bottleneck_capacity_edges.append(
                    self.graph.get_edge((path[index], path[index + 1]))
                )
            path_capacity = min(
                min(bottleneck_capacity_edges), bottleneck_capacity
            )

            completion_time = (
                total_turns + ceil(drones_in_path / path_capacity) - 1
            )

            if completion_time < min_turns:
                min_turns = completion_time
                best_path = path

        return best_path, min_turns
