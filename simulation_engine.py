from sys import maxsize
from typing import List, Dict, Tuple, Protocol
from itertools import combinations
from math import ceil
from input_parser import Node, NetworkParser
from graph import Graph
from min_cost_max_flow import Path, MinCostMaxFlowAlgorithm
from dataclasses import dataclass

#make type for tuple(str-str)


@dataclass
class Drone:
    id: int
    moved: bool = False
    turns: int = 0


class PathsFindingAlgorithm(Protocol):
    def find_paths(self, graph: Graph) -> List[Path]:
        ...


class RoutePlanner: #chooses best path/s and assings drones to it/them
    paths: List[Path]
    drones: int
    nodes: Dict[str, Node]
    connections: Dict[Tuple[str, str], int]

    def __init__(self, paths_finding_algorithm: PathsFindingAlgorithm, graph: Graph) -> None:
        self.paths = paths_finding_algorithm.find_paths(graph)
        self.drones = graph.drones
        self.nodes = graph.get_all_nodes()
        self.connections = graph.get_all_edges()

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

        for id in range(1, self.drones + 1):
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

        for n, path in enumerate(paths):
            # print(f"path stats for path {path}")
            drones_in_path = drones_per_path[n] + 1

            total_turns = 0
            # print(f"path is {path[1:]}")
            for node in path[1:]: #skips start???
                if self.nodes[node].zone.value == "restricted":
                    total_turns += 2
                else:
                    total_turns += 1

            # print(f"total_turns {total_turns}")
            bottleneck_capacity = min([
                self.nodes[node].max_drones for node in path
            ])
            bottleneck_capacity_edges = []
            for n in range(0, len(path) - 1):
                bottleneck_capacity_edges.append(
                    self.connections[(path[n], path[n + 1])]
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


class SimulationEngine:
    paths: List[Path]
    drone_assignment: List[List[Drone]]
    turns: int
    start: str
    end: str
    nodes: Dict[str, Node]
    connections: Dict[Tuple[str, str], int]
    metrics: "OptionalMetrics"

    def __init__(
        self,
        paths: List[Path],
        drone_assignment: List[List[Drone]],
        turns: int,
        graph: Graph
    ) -> None:
        self.paths = SimulationEngine.insert_edges(paths)
        self.drone_assignment = drone_assignment
        self.turns = turns
        self.start = graph.start.name
        self.end = graph.end.name
        self.nodes = graph.get_all_nodes()
        self.connections = graph.get_all_edges()
        self.metrics = self.OptionalMetrics()

    @staticmethod
    def insert_edges(paths: List[Path]) -> List[Path]:
        path_with_edges: List[Path] = []
        for path in paths:
            new_path: Path = []
            for n in range(0, len(path)):
                new_path.append(path[n])
                if n < len(path) - 1:
                    new_path.append(path[n] + "-" + path[n + 1])
            path_with_edges.append(new_path)

        return path_with_edges

    def simulate_turns(self) -> None: #Dict[int, Dict[str, List[str]]]: #per turn per location the list of drones
        # result: Dict[int, Dict[str, List[Drone]]] = {} #might not need it?????
        paths_state: List[Dict[str, List[Drone]]] = [
            {point: [] for point in path} for path in self.paths
        ]

        for n, path in enumerate(paths_state):
            path[self.start] = self.drone_assignment[n]

        for _ in range(1, self.turns + 1):
            for n, path in enumerate(self.paths):
                reversed_path = list(reversed(path))
                for point in reversed_path:
                    if point == self.end:
                        continue
                    if paths_state[n][point] != []:
                        next_point, drones = self.get_next_move(path, point)
                        SimulationEngine.move_drones(
                            paths_state[n], point, next_point, drones
                        )
            # result[turn] = SimulationEngine.consolidate_paths(paths_state)
            self.print_simulation_output(
                SimulationEngine.consolidate_paths(paths_state)
            )

            self.metrics.add_drones_moved(
                [
                    drone
                    for path in paths_state
                    for drones in path.values()
                    for drone in drones
                ]
            )

            for path in paths_state:
                for point in path.keys():
                    SimulationEngine.switch_drone_status(path[point], False)

            self.metrics.calculate_turns_per_drone(
                SimulationEngine.consolidate_paths(paths_state)[self.end]
            )
        # return result

    def get_next_move(self, path: Path, current: str) -> Tuple[str, int]:
        current_index: int = path.index(current)
        if self.nodes.get(path[current_index + 1]) is None:
            from_node, to_node = path[current_index + 1].split("-")
            edge = (from_node, to_node)
            edge_drones = self.connections.get(edge)
            if self.nodes[path[current_index + 2]].zone.value == "restricted":
                next_point = path[current_index + 1]
                drones = edge_drones
            else:
                next_point = path[current_index + 2]
                drones = min(self.nodes[next_point].max_drones, edge_drones)
        else:
            next_point = path[current_index + 1]
            drones = self.nodes[next_point].max_drones

        return (next_point, drones)

    @staticmethod
    def move_drones(
        state: Dict[str, List[Drone]],
        from_point: str,
        to_point: str,
        drones: int
    ) -> None:
        state[to_point].extend(state[from_point][:drones])
        drones_to_move: List[Drone] = state[from_point][:drones]
        SimulationEngine.switch_drone_status(drones_to_move, True)
        # for drone in state[from_point][:drones]:
        #     drone.moved = True
        del state[from_point][:drones]

    @staticmethod
    def consolidate_paths(
        paths_state: List[Dict[str, List[Drone]]]
    ) -> Dict[str, List[Drone]]:
        consolidated_path: Dict[str, List[str]] = {}

        for path in paths_state:
            for point, drones in path.items():
                if consolidated_path.get(point) is None:
                    consolidated_path[point] = list(drones)
                else:
                    consolidated_path[point].extend(drones)

        return consolidated_path

    def print_simulation_output(self, output: Dict[str, List[Drone]]) -> None:
        formatted_output: List = []

        for point, drones in output.items():
            for drone in drones:
                if point == self.start or not drone.moved:
                    continue
                formatted_output.append(drone.id + "-" + point)

        print((" ").join(sorted(formatted_output)))

    @staticmethod
    def switch_drone_status(drones: List[Drone], moved: bool):
        for drone in drones:
            drone.moved = moved
            if moved:
                drone.turns += 1

    def print_secondary_metrics(self) -> None:
        print("\nNumber of drones moved per turn:")
        for n in range(len(self.metrics.get_drones_per_turn())):
            print(
                f"Turn {n + 1}: {self.metrics.get_drones_per_turn()[n]} drones"
            )

        av_turns = self.metrics.get_av_turns_per_drone()
        print(f"\nAverage number of turns per drone: {av_turns:.2f}")

    class OptionalMetrics:
        drones: int
        drones_per_turn: List[int]
        turns_per_drone: int

        def __init__(self) -> None:
            self.drones = 0
            self.drones_per_turn = []
            self.turns_per_drone = 0

        def add_drones_moved(self, drones: List[Drone]) -> None:
            self.drones_per_turn.append(sum(drone.moved for drone in drones))

        def calculate_turns_per_drone(self, drones: List[Drone]) -> None:
            self.drones += len(drones)
            self.turns_per_drone += sum(drone.turns for drone in drones)

        def get_drones_per_turn(self) -> List[int]:
            return self.drones_per_turn

        def get_av_turns_per_drone(self) -> float:
            return self.turns_per_drone / self.drones


if __name__ == "__main__":
    try:
        graph = NetworkParser.load("03_priority_puzzle.txt").to_graph()

        path_finding_algorithm = MinCostMaxFlowAlgorithm()

        subset = RoutePlanner(path_finding_algorithm, graph)
        path, drone_assignment, turns = subset.schedule_drones()
        # print(path)
        # print(drone_assignment)
        # print(turns)
        simulation = SimulationEngine(path, drone_assignment, turns, graph)
        simulation.simulate_turns()
        simulation.print_secondary_metrics()
        # for turn, drones in turns.items():
        #     print(f"turn {turn} has {drones}\n")
        # MinCostMaxFlow.run(flow_graph.flow_graph), graph.get_all_nodes(), 5, graph.get_connections()
        # print(repr(flow_graph.flow_graph))
        # print()
        # PathSelection(MinCostMaxFlow.find_cheapest_paths(flow_graph.flow_graph), graph.get_all_nodes(), 5, graph.get_connections())
    except ValueError as e:
        print(f"Error {e}")

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
# Scheduler
#     choose_best_subset()
#     assign_drones()
#     simulate_turns()