from typing import List, Dict, Tuple, Optional
import copy
from math import ceil
from input_parser import Node, NetworkParser
from graph import Graph
from min_cost_max_flow import Path, MinCostMaxFlowAlgorithm
from route_planner import Drone, RoutePlanner
from renderer import Renderer


class SimulationEngine:
    paths: List[Path]
    drone_assignment: List[List[Drone]]
    turns: int
    graph: Graph
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
        self.graph = graph
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
        simulation_history: Dict[int, Dict[str, List[Drone]]] = {} #might not need it?????
        current_occupancy: Dict[str, int] = self.reset_occupancy()
        all_drones: List[Drone] = sorted(
            list(
                drone
                for drones_per_path in self.drone_assignment
                for drone in drones_per_path
            ),
            key=lambda drone: drone.id
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
        # print(simulation_history[0])

        for turn in range(1, self.turns + 1):
            for drone in all_drones:
                if drone.current != end:
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
                            drone.current,
                            current_occupancy
                        ),
                        drone
                    )

            current_occupancy.update(self.reset_occupancy())
            simulation_history[turn] = SimulationEngine.consolidate_paths(
                copy.deepcopy(paths_state)
            )
            self.print_simulation_output(simulation_history[turn])

        #     for index, path in enumerate(self.paths):
        #         reversed_path = list(reversed(path))
        #         for point in reversed_path:
        #             if point == end:
        #                 continue
        #             if paths_state[index][point] != []:
        #                 next_point, drones = self.get_next_move(path, point)
        #                 self.move_drones(
        #                     paths_state[index], point, next_point, drones
        #                 )
        #     simulation_history[turn] = SimulationEngine.consolidate_paths(
        #         copy.deepcopy(paths_state)
        #     )
        #     self.print_simulation_output(simulation_history[turn])
        #     # self.print_simulation_output(
        #     #     SimulationEngine.consolidate_paths(paths_state)
        #     # )

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

        #     for path in paths_state:
        #         for point in path.keys():
        #             SimulationEngine.switch_drone_status(path[point], False)

        # # for turn, data in simulation_history.items():
        # #     print(f"turn {turn}: {data}")
        # #     print()
        self.metrics.add_turns_per_drone(simulation_history[self.turns][end])
        # self.metrics.add_turns_per_drone(
        #     SimulationEngine.consolidate_paths(paths_state)[end]
        # )
        Renderer.render_simulation(self.graph, simulation_history)
        # # return result

    def reset_occupancy(self) -> Dict[str, int]:
        return {
            point: (
                self.graph.get_node(point).max_drones
                if point in self.graph.get_all_nodes().keys()
                else self.graph.get_all_edges()[
                    (point.split("-")[0], point.split("-")[1])
                ]
            )
            for path in self.paths
            for point in path
        }

    def get_next_move(
        self, path: Path, current: str, current_occupancy: Dict[str, int]
    ) -> Optional[str]:
        nodes: Dict[str, Node] = self.graph.get_all_nodes()
        current_index: int = path.index(current)

        if nodes.get(path[current_index + 1]) is None:
            if nodes[path[current_index + 2]].zone.value == "restricted":
                next_point = path[current_index + 1]
            else:
                next_point = path[current_index + 2]
        else:
            next_point = path[current_index + 1]

        if current_occupancy[next_point] == 0:
            return None

        current_occupancy[next_point] -= 1
        return next_point

    # def get_next_move(self, path: Path, current: str) -> Tuple[str, int]:
    #     nodes: Dict[str, Node] = self.graph.get_all_nodes()
    #     current_index: int = path.index(current)

    #     if nodes.get(path[current_index + 1]) is None:
    #         from_node, to_node = path[current_index + 1].split("-")
    #         edge = (from_node, to_node)
    #         edge_drones = self.graph.get_all_edges().get(edge)
    #         if nodes[path[current_index + 2]].zone.value == "restricted":
    #             next_point = path[current_index + 1]
    #             drones = edge_drones
    #         else:
    #             next_point = path[current_index + 2]
    #             drones = min(nodes[next_point].max_drones, edge_drones)
    #     else:
    #         next_point = path[current_index + 1]
    #         drones = nodes[next_point].max_drones

    #     return (next_point, drones)

    def move_drones(
        self,
        state: Dict[str, List[Drone]],
        from_point: str,
        to_point: str,
        drone: Drone
    ) -> None:
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
            del state[from_point][current_index]#always removes first drone anyway???

    # def move_drones(
    #     self,
    #     state: Dict[str, List[Drone]],
    #     from_point: str,
    #     to_point: str,
    #     drones: int,
    #     marked_as_moved: bool = True
    # ) -> None:
    #     state[to_point].extend(state[from_point][:drones])
    #     drones_to_move: List[Drone] = state[from_point][:drones]

    #     for drone in drones_to_move:
    #         if "-" in to_point:
    #             drone.x, drone.y = self.calculate_edge_coordinate(to_point)
    #         else:
    #             drone.x = self.graph.get_node(to_point).x
    #             drone.y = self.graph.get_node(to_point).y

    #     if marked_as_moved:
    #         SimulationEngine.switch_drone_status(drones_to_move, True)

    #     del state[from_point][:drones]

    def calculate_edge_coordinate(self, to_point: str) -> Tuple[int, int]:
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
        consolidated_path: Dict[str, List[Drone]] = {}

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
                if point == self.graph.get_start().name or not drone.moved:
                    continue
                formatted_output.append(drone.id + "-" + point)

        print((" ").join(sorted(formatted_output)))

    @staticmethod
    def switch_drone_status(drone: Drone, moved: bool):
        drone.moved = moved
        if moved:
            drone.turns += 1

    # @staticmethod
    # def switch_drone_status(drones: List[Drone], moved: bool):
    #     for drone in drones:
    #         drone.moved = moved
    #         if moved:
    #             drone.turns += 1

    def print_secondary_metrics(self) -> None:
        print("Number of drones moved per turn:")
        for n in range(len(self.metrics.get_drones_per_turn())):
            print(
                f"Turn {n + 1}: {self.metrics.get_drones_per_turn()[n]} drones"
            )

        print("\nNumber of turns per drone:")
        for id, turns in self.metrics.get_turns_per_drone().items():
            print(f"Drone '{id}': {turns} turns")

        av_turns = self.metrics.get_av_turns_per_drone()
        print(f"\nAverage number of turns per drone: {av_turns}")

    class OptionalMetrics:
        drones_per_turn: List[int]
        turns_per_drone: Dict[str, int]

        def __init__(self) -> None:
            self.drones_per_turn = []
            self.turns_per_drone = {}

        def add_drones_moved_in_turn(self, drones: List[Drone]) -> None:
            self.drones_per_turn.append(sum(drone.moved for drone in drones))

        def get_drones_per_turn(self) -> List[int]:
            return self.drones_per_turn

        def add_turns_per_drone(self, drones: List[Drone]) -> None:
            self.turns_per_drone.update(
                {drone.id: drone.turns for drone in drones}
            )

        def get_turns_per_drone(self) -> Dict[str, int]:
            return {
                drone: turns
                for drone, turns in sorted(
                    self.turns_per_drone.items(), key=lambda item: item[0]
                )
            }

        def get_av_turns_per_drone(self) -> int:
            return ceil(
                sum(turns for turns in self.turns_per_drone.values()) /
                len(self.turns_per_drone.keys())
            )


if __name__ == "__main__":
    try:
        graph = NetworkParser.load("./maps/easy/02_simple_fork.txt").to_graph()

        path_finding_algorithm = MinCostMaxFlowAlgorithm()

        subset = RoutePlanner(path_finding_algorithm, graph)
        path, drone_assignment, turns = subset.schedule_drones()
        # print(path)
        # print(drone_assignment)
        # print(turns)
        simulation = SimulationEngine(path, drone_assignment, turns, graph)
        print()
        simulation.simulate_turns()
        print()
        simulation.print_secondary_metrics()
        print()

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