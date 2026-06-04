from typing import List, Dict, Tuple
from input_parser import Zone, NetworkParser, Connection
from flow_graph import Edge
from graph import Graph
from flow_graph import FlowGraph
from itertools import combinations
from math import ceil

class MinCostMaxFlow:
    @staticmethod
    def find_cheapest_paths( #change this to __init__? name is wrong anyway...
        flow_graph: Dict[str, List[Edge]]
    ) -> List[List[str]]:
        start: str = list(flow_graph.keys())[0]
        goal: str = list(flow_graph.keys())[-1]
        total_flow: int = 0
        total_cost: int = 0

        # print(f"flow graph before: {flow_graph}")
        while True:
            result = MinCostMaxFlow.bellman_ford(flow_graph, start, goal)

            if result is None:
                break

            predecessors, path_cost = result

            amount = MinCostMaxFlow.augment(
                flow_graph, start, goal, predecessors
            )

            total_flow += amount
            total_cost += amount * path_cost

            # for k, v in result[0].items():
            #     print(f"{k}: {v}\n")
            # print()
            # print(amount)

        if total_flow == 0:
            raise ValueError("End of network unreachable!")

        # print(f"flow graph after: {flow_graph}")

        return MinCostMaxFlow.decompose_paths(flow_graph, start, goal)

    @staticmethod
    def bellman_ford(
        flow_graph: Dict[str, List[Edge]], start: str, goal: str
    ) -> Tuple[Dict[str, Tuple[str, int]], int]:
        distances: Dict[str, int] = {
            node: float("inf") for node in flow_graph.keys()
        }
        predecessors: Dict[str, Tuple[str, int]] = {
            node: (None, 0) for node in flow_graph.keys()
        }

        distances[start] = 0
        for _ in range(len(flow_graph) - 1):
            updated = False

            for node in flow_graph.keys():
                if distances[node] == float("inf"):
                    continue

                for i, edge in enumerate(flow_graph[node], start=0):
                    if edge.capacity <= 0:
                        continue

                    new_distance = distances[node] + edge.cost
                    if new_distance < distances[edge.to]:
                        distances[edge.to] = new_distance
                        predecessors[edge.to] = (node, i)
                        updated = True

            if not updated:
                break

        for node in flow_graph.keys():
            if distances[node] == float("inf"):
                continue

            for i, edge in enumerate(flow_graph[node], start=0):
                if edge.capacity <= 0:
                    continue

                if distances[node] + edge.cost < distances[edge.to]:
                    raise ValueError("Negative cycle detected.")

        if predecessors[goal][0] is None: #maybe better to raise error instead of total_flow == 0 aboce
            return None
        return predecessors, distances[goal]

    @staticmethod
    def augment(
        flow_graph: Dict[str, List[Edge]],
        start: str,
        goal: str,
        predecessors: Dict[str, Tuple[str, int]]
    ) -> int:
        amount: int = float("inf")
        node: str = goal

        while node != start:
            previous, i = predecessors[node]
            edge = flow_graph[previous][i]
            amount = min(amount, edge.capacity)
            node = previous

        node = goal
        while node != start:
            previous, i = predecessors[node]
            edge = flow_graph[previous][i]
            reverse = flow_graph[edge.to][edge.reverse]
            edge.capacity -= amount
            reverse.capacity += amount
            node = previous

        return amount

    @staticmethod
    def decompose_paths(
        flow_graph: Dict[str, List[Edge]], start: str, goal: str
    ) -> List[List[str]]:
        paths: List[List[str]] = []
        remaining_flow: Dict[Tuple[str, int], int] = {}

        for node, edges in flow_graph.items():
            for i, edge in enumerate(edges, start=0):
                if edge.original:
                    used = flow_graph[edge.to][edge.reverse].capacity
                    if used > 0:
                        remaining_flow[(node, i)] = used

        while True:
            path = [start]
            node = start
            visited = set()

            while node != goal:
                if node in visited:
                    raise ValueError("Cycle while decomposing.")
                visited.add(node)

                next_item = None

                for i, edge in enumerate(flow_graph[node], start=0):
                    if edge.original and remaining_flow.get((node, i), 0) > 0:
                        next_item = (i, edge)
                        break

                if next_item is None:
                    if node == start:
                        return paths
                    raise ValueError("Broken flow: path stopped before goal.")

                i, edge = next_item
                remaining_flow[(node, i)] -= 1
                node = edge.to
                path.append(node)

            paths.append(path)


class SubsetSelection:
    best_subset: List[List[str]]
    best_assignment: List[str] #list of drones per list of subset (best paths)

    def __init__(self, paths: List[List[str]], nodes: Dict[str, Zone], nb_drones: int, connections: List[Connection]) -> None:
        self.paths = paths
        self.drones = nb_drones
        self.best_subset = []
        min_turns = float("inf")
        self.best_assignment = []

        collapsed_paths = SubsetSelection.collapse_paths(paths)
        for size in range(1, len(collapsed_paths) + 1):
            for subset in combinations(collapsed_paths, size):
                print(f"check subset {subset}")
                turns, assignment = SubsetSelection.drone_assignment(list(subset), self.drones, nodes, connections)
                print(f"has turns {turns} and list of drones {assignment}")

                if turns < min_turns:
                    min_turns = turns
                    self.best_subset = list(subset)
                    self.best_assignment = assignment

        print(self.best_subset, self.best_assignment, min_turns)

    @staticmethod
    def collapse_paths(subset: List[List[str]]) -> List[List[str]]:
        result = []
        for set in subset:
            new_set = []
            for node in set:
                if node.endswith("_in"):
                    og_node = node[:-3]
                elif node.endswith("_out"):
                    og_node = node[:-4]
                if og_node not in new_set:
                    new_set.append(og_node)
            result.append(new_set)
        return result

    @staticmethod
    def drone_assignment(paths: List[List[str]], nb_drones: int, nodes: Dict[str, Zone], connections: List[Connection]) -> Tuple[int, List[str]]:
        n_drones_per_path = [0 for path in paths]
        id_drones_per_path = [[] for path in paths]
        turns_per_path = [0 for path in paths]

        for id in range(1, nb_drones + 1):
            best_path, turns = SubsetSelection.choose_best_path(paths, n_drones_per_path, nodes, connections)
            id_drones_per_path[paths.index(best_path)].append("drone_" + str(id))
            n_drones_per_path[paths.index(best_path)] += 1
            turns_per_path[paths.index(best_path)] = turns
        
        return (max(turns_per_path), id_drones_per_path)

    @staticmethod
    def choose_best_path(paths: List[List[str]], drones_per_path: List[int], nodes: Dict[str, Zone], connections: List[Connection]) -> Tuple[int, List[str]]:
        best_path = None
        min_turns = float("inf")

        for n, path in enumerate(paths, start=0):
            # print(f"path stats for path {path}")
            drones_in_path = drones_per_path[n] + 1

            total_turns = 0
            for node in path[1:]:
                if nodes[node].zone.value == "restricted":
                    total_turns += 2
                else:
                    total_turns += 1
            
            bottleneck_capacity_nodes = min([nodes[node].max_drones for node in path])
            connections_dic = {(connection.from_hub, connection.to_hub): connection.max_link_capacity for connection in connections}
            bottleneck_capacity_edges = []
            for n in range(0, len(path) - 1):
                bottleneck_capacity_edges.append(connections_dic[(path[n], path[n + 1])])
            path_capacity = min(min(bottleneck_capacity_edges), bottleneck_capacity_nodes)

            completion_time = total_turns + ceil(drones_in_path / path_capacity) - 1
            # print(f"completion_time {completion_time}")
            # print(f"path_duration{total_turns}, path_capacity{path_capacity} completion_time {completion_time}")

            if completion_time < min_turns:
                min_turns = completion_time
                best_path = path
            
        return best_path, min_turns


# class SSAP:

if __name__ == "__main__":
    try:
        network = NetworkParser.load("03_priority_puzzle.txt")
        graph = Graph(network)
        # Renderer().render_network(graph.get_graph(), graph.get_nodes())
        flow_graph = FlowGraph(graph.get_connections(), graph.get_nodes())
        # print(repr(flow_graph.flow_graph))
        # print()
        SubsetSelection(MinCostMaxFlow.find_cheapest_paths(flow_graph.flow_graph), graph.get_nodes(), 5, graph.get_connections())
    except ValueError as e:
        print(f"Error {e}")

# 1. Parse graph
# 2. Build flow network with room capacities/costs
# 3. Run Min-Cost Max-Flow using SSAP + Bellman-Ford
# 4. Decompose final flow into independent paths
# 5. Choose best subset of paths
# 6. Assign drones by path length/load
# 7. Simulate turns simply

# while there is a cheapest path from source to sink:
#     run Bellman-Ford
#         relax edges up to V - 1 times
#         find cheapest augmenting path in residual graph

#     if sink is unreachable:
#         stop

#     augment flow along that path
#     update residual capacities

# Bellman-Ford Uses Capacity
# Bellman-Ford searches only usable residual edges:

# if edge.capacity <= 0:
#     continue
# Cost decides which available path is cheapest. Capacity decides whether an edge is available at all.



# 1. Min-Cost Max-Flow
#    Finds independent candidate lanes.

# 2. Path decomposition
#    Extracts actual paths from final positive flow.

# 3. Path subset selection
#    Decides which extracted paths are worth using.

# 4. Drone assignment
#    Assigns drones to the chosen paths.

# 5. Turn simulation
#    Moves drones over time.


# Bellman-Ford:
#     chooses the cheapest currently available augmenting path
#     this is where costs matter

# Augmentation:
#     reserves/uses capacity along that path
#     this is where capacities matter

# Decomposition:
#     extracts the final used flow paths
#     it does not decide preference


# This specific approach is mostly the classic lem-in path scheduling heuristic.

# The theory names around it are:

# Network Flow
# for finding independent paths / capacities.

# Min-Cost Max-Flow
# for finding good capacity-respecting paths with costs/preferences.

# Path decomposition
# for extracting source-to-sink routes from final flow.

# Load balancing / makespan minimization
# for assigning drones across paths to minimize the last arrival time.

# The exact drone assignment problem is closest to:

# Scheduling on parallel machines
# More specifically:

# makespan minimization
# because you are minimizing:

# the maximum finish time among all paths
# In your case, each path is like a “machine” with:

# startup time = path_duration
# throughput = path_capacity
# Then you assign drones/jobs to paths to minimize the final completion time.

# So if you want search terms:

# lem-in path scheduling
# network flow path decomposition
# min-cost max-flow path decomposition
# load balancing minimize makespan
# parallel machine scheduling makespan
# flow over time
# dynamic network flow
# For a more mathematically exact version of “drones moving over turns with capacities,” the term is:

# flow over time
# or:

# dynamic network flow
# But the practical 42-style version is:

# Min-Cost Max-Flow + path decomposition + makespan/load-balancing assignment
