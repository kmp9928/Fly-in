from sys import maxsize
from typing import List, Tuple, Protocol
from itertools import combinations
from math import ceil
from dataclasses import dataclass
from graph import Graph
from min_cost_max_flow import Path


@dataclass
class Drone:
    id: int
    current: str = None
    x: int = 0
    y: int = 0
    moved: bool = False
    turns: int = 0


class PathsFindingAlgorithm(Protocol):
    def find_paths(self, graph: Graph) -> List[Path]:
        ...


class RoutePlanner: #chooses best path/s and assings drones to it/them
    paths: List[Path]
    graph: Graph

    def __init__(
        self, paths_finding_algorithm: PathsFindingAlgorithm, graph: Graph
    ) -> None:
        self.paths = paths_finding_algorithm.find_paths(graph)
        self.graph = graph

    def schedule_drones(self) -> Tuple[List[Path], List[List[Drone]], int]:
        best_subset: List[Path] = []
        best_assignment: List[List[Drone]] = [] #list of drones per list of subset (best paths)
        min_turns: int = maxsize

        for size in range(1, len(self.paths) + 1):
            for subset in combinations(self.paths, size):
                # print(f"subset to check {subset}")
                turns, assignment = self.drone_assignment(list(subset))
                # print(f"has turns {turns} and list of drones {assignment}")

                if turns < min_turns:
                    min_turns = turns
                    best_subset = list(subset)
                    best_assignment = assignment

        return (best_subset, best_assignment, min_turns)

    def drone_assignment(self, paths: List[Path]) -> Tuple[int, List[Drone]]:
        n_drones_per_path: List[int] = [0 for _ in paths]
        drones_per_path: List[List[Drone]] = [[] for _ in paths]
        turns_per_path: List[int] = [0 for _ in paths]

        for id in range(1, self.graph.get_drones_n() + 1):
            best_path, turns = self.choose_best_path(
                paths, n_drones_per_path
            )
            drone = Drone(id="D" + str(id))
            best_path_index = paths.index(best_path)
            drones_per_path[best_path_index].append(drone)
            n_drones_per_path[best_path_index] += 1
            turns_per_path[best_path_index] = turns

        return (max(turns_per_path), drones_per_path)

    def choose_best_path(
        self, paths: List[Path], drones_per_path: List[int]
    ) -> Tuple[Path, int]:
        best_path: Path = [] #None
        min_turns: int = maxsize

        for index, path in enumerate(paths):
            # print(f"path stats for path {path}")
            drones_in_path = drones_per_path[index] + 1

            total_turns = 0
            # print(f"path is {path[1:]}")
            for node in path:
                if node == self.graph.get_start().name:   #skips start???
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
                    self.graph.get_all_edges()[(path[index], path[index + 1])]
                )
                # bottleneck_capacity_edges.append(connections_dic[(path[n], path[n + 1])])
            path_capacity = min(
                min(bottleneck_capacity_edges), bottleneck_capacity
            )

            completion_time = (
                total_turns + ceil(drones_in_path / path_capacity) - 1
            )
            # print(f"completion_time {completion_time}")
            # print(f"path_duration{total_turns}, path_capacity{path_capacity} completion_time {completion_time}")

            if completion_time < min_turns:
                min_turns = completion_time
                best_path = path

        return best_path, min_turns
