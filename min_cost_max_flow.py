from sys import maxsize
from typing import List, Dict, Tuple
from dataclasses import dataclass
from input_parser import Node
from errors import FlowGraphError
from graph import Graph, ConnectingNodes


#make type of tuple [str, int]


@dataclass
class Edge:
    to: str
    capacity: int
    cost: int
    reverse: int = 0
    original: bool = False


class FlowGraph:
    start: str
    end: str
    flow_graph: Dict[str, List[Edge]]
    forbidden: List[str]

    def __init__(self, graph: Graph) -> None:
        self.start = FlowGraph.make_out_name(graph.get_start().name)
        self.end = FlowGraph.make_in_name(graph.get_end().name)
        self.flow_graph = {}
        self.forbidden = [
            FlowGraph.make_in_name(graph.start.name),
            FlowGraph.make_out_name(graph.end.name)
        ]

        for connection, capacity in graph.get_all_edges().items():
            self.add_edge_pair(connection, capacity)
            self.add_residual_pair(connection)

        for node in graph.get_all_nodes().values():
            if node.zone.value != "blocked":
                self.add_inner_pair(node)

        self.add_reverse_index()

    @staticmethod
    def make_in_name(name: str) -> str:
        return name + "_in"

    @staticmethod
    def make_out_name(name: str) -> str:
        return name + "_out"

    def add_edge_pair(self, edges: ConnectingNodes, capacity: int) -> None:
        self.add_edge(
            FlowGraph.make_out_name(edges[0]),
            FlowGraph.make_in_name(edges[1]),
            capacity,
            1,
            True
        )
        self.add_edge(
            FlowGraph.make_out_name(edges[1]),
            FlowGraph.make_in_name(edges[0]),
            capacity,
            1,
            True
        )

    def add_residual_pair(self, edges: ConnectingNodes) -> None:
        self.add_edge(
            FlowGraph.make_in_name(edges[0]),
            FlowGraph.make_out_name(edges[1]),
            0,
            -1
        )
        self.add_edge(
            FlowGraph.make_in_name(edges[1]),
            FlowGraph.make_out_name(edges[0]),
            0,
            -1
        )

    def add_inner_pair(self, node: Node) -> None:
        node_costs: Dict[str, int] = {
            "normal": 1,
            "restricted": 10,
            "priority": 0
        }

        self.add_edge(
            FlowGraph.make_in_name(node.name),
            FlowGraph.make_out_name(node.name),
            node.max_drones,
            node_costs[node.zone.value],
            True
        )
        self.add_edge(
            FlowGraph.make_out_name(node.name),
            FlowGraph.make_in_name(node.name),
            0,
            -node_costs[node.zone.value],
        )

    def is_forbidden(self, node_name: str) -> bool:
        return node_name in self.forbidden

    def add_edge(
        self,
        from_id: str,
        to_id: str,
        capacity: int,
        cost: int,
        original: bool = False
    ) -> None:
        edge = Edge(
            to=to_id,
            capacity=capacity,
            cost=cost,
            original=original
        )
        if not self.is_forbidden(from_id) and not self.is_forbidden(to_id):
            if self.flow_graph.get(from_id) is None:
                self.flow_graph[from_id] = [edge]
            else:
                self.flow_graph[from_id].extend([edge])

    def add_reverse_index(self) -> None:
        for from_id, edges in self.flow_graph.items():
            for edge in edges:
                for i, edges_of_reverse in enumerate(self.flow_graph[edge.to]):
                    if edges_of_reverse.to == from_id:
                        break
                edge.reverse = i

    def get_nodes(self) -> List[str]:
        return list(self.flow_graph.keys())

    def get_edges(self, node: str) -> List[Edge]:
        return self.flow_graph[node]

    def get_start(self) -> str:
        return self.start

    def get_end(self) -> str:
        return self.end


type Path = List[str]


class MinCostMaxFlowAlgorithm:
    def find_paths(self, graph: Graph) -> List[Path]:
        flow_graph = FlowGraph(graph)
        total_flow: int = 0
        total_cost: int = 0

        while True:
            result = MinCostMaxFlowAlgorithm.bellman_ford(flow_graph)

            if result is None:
                break

            predecessors, path_cost = result
            amount = MinCostMaxFlowAlgorithm.augment(flow_graph, predecessors)

            total_flow += amount
            total_cost += amount * path_cost

        if total_flow == 0:
            raise FlowGraphError("End of network unreachable!")

        return MinCostMaxFlowAlgorithm.collapse_paths(
            MinCostMaxFlowAlgorithm.decompose_paths(flow_graph)
        )

    def bellman_ford(
        flow_graph: FlowGraph
    ) -> Tuple[Dict[str, Tuple[str, int]], int]:
        nodes: List[str] = flow_graph.get_nodes()
        distances: Dict[str, int] = {
            node: maxsize for node in nodes
        }
        predecessors: Dict[str, Tuple[str, int]] = {
            node: (None, 0) for node in nodes
        }
        distances[flow_graph.get_start()] = 0

        for _ in range(len(nodes) - 1):
            updated = False

            for node in nodes:
                if distances[node] == maxsize:
                    continue

                for i, edge in enumerate(flow_graph.get_edges(node)):
                    if edge.capacity <= 0:
                        continue

                    new_distance = distances[node] + edge.cost
                    if new_distance < distances[edge.to]:
                        distances[edge.to] = new_distance
                        predecessors[edge.to] = (node, i)
                        updated = True

            if not updated:
                break

        for node in nodes:
            if distances[node] == maxsize:
                continue

            for i, edge in enumerate(flow_graph.get_edges(node)):
                if edge.capacity <= 0:
                    continue

                if distances[node] + edge.cost < distances[edge.to]:
                    raise FlowGraphError("Negative cycle detected.")

        if predecessors[flow_graph.get_end()][0] is None:
            return None

        return predecessors, distances[flow_graph.get_end()]

    def augment(
        flow_graph: FlowGraph, predecessors: Dict[str, Tuple[str, int]]
    ) -> int:
        amount: int = maxsize
        node: str = flow_graph.get_end()

        while node != flow_graph.get_start():
            previous, i = predecessors[node]
            edge = flow_graph.get_edges(previous)[i]
            amount = min(amount, edge.capacity)
            node = previous

        node = flow_graph.get_end()
        while node != flow_graph.get_start():
            previous, i = predecessors[node]
            edge = flow_graph.get_edges(previous)[i]
            reverse = flow_graph.get_edges(edge.to)[edge.reverse]
            edge.capacity -= amount
            reverse.capacity += amount
            node = previous

        return amount

    def decompose_paths(flow_graph: FlowGraph) -> List[Path]:
        paths: List[Path] = []
        remaining_flow: Dict[Tuple[str, int], int] = {}

        for node in flow_graph.get_nodes():
            for i, edge in enumerate(flow_graph.get_edges(node)):
                if edge.original:
                    reverse_edges = flow_graph.get_edges(edge.to)
                    used = reverse_edges[edge.reverse].capacity
                    if used > 0:
                        remaining_flow[(node, i)] = used

        while True:
            path = [flow_graph.get_start()]
            node = flow_graph.get_start()
            visited = set()

            while node != flow_graph.get_end():
                if node in visited:
                    raise FlowGraphError("Cycle while decomposing.")

                visited.add(node)
                next_item = None

                for i, edge in enumerate(flow_graph.get_edges(node)):
                    if edge.original and remaining_flow.get((node, i), 0) > 0:
                        next_item = (i, edge)
                        break

                if next_item is None:
                    if node == flow_graph.get_start():
                        return paths
                    raise FlowGraphError(
                        "Broken flow path stopped before goal."
                    )

                i, edge = next_item
                remaining_flow[(node, i)] -= 1
                node = edge.to
                path.append(node)

            paths.append(path)

    @staticmethod
    def collapse_paths(paths: List[Path]) -> List[Path]:
        result = []
        for path in paths:
            new_path = []
            for node in path:
                if node.endswith("_in"):
                    original_node = node[:-3]
                elif node.endswith("_out"):
                    original_node = node[:-4]
                if original_node not in new_path:
                    new_path.append(original_node)
            result.append(new_path)

        return result


# if __name__ == "__main__":
#     try:
#         network = NetworkParser.load("03_priority_puzzle.txt")
#         graph = network.to_graph()
#         flow_graph = FlowGraph(graph)
#         mcmf = MinCostMaxFlow(flow_graph)
#         paths = mcmf.run()
#         for path in paths:
#             print(f"One possible path is: {path}")
#         # MinCostMaxFlow.run(flow_graph.flow_graph), graph.get_all_nodes(), 5, graph.get_connections()
#     except ValueError as e:
#         print(f"Error {e}")

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# class SSAP:

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



# NetworkParser
#     reads/validates input

# Graph
#     stores original hubs/connections/nb_drones
#     maybe provides neighbor helpers

# FlowGraph
#     builds residual graph from Graph
#     add_edge_pair()
#     add_room_edges()
#     add_tunnel_edges()

# MinCostMaxFlow
#     bellman_ford()
#     augment()
#     decompose_paths()

# Scheduler
#     choose_best_subset()
#     assign_drones()
#     simulate_turns()


# decompose_paths takes the final flow result from Min-Cost Max-Flow and turns it into actual usable paths.

# During the algorithm, Bellman-Ford finds augmenting paths and augment() updates residual capacities. But those augmenting paths are not necessarily the final drone routes, because later augmentations can reroute earlier flow.

# So after the algorithm finishes, decompose_paths looks at the residual graph and asks:

# Which original edges ended up carrying positive flow?
# Usually that is detected with:

# used_flow = reverse_edge.capacity
# Then it follows original edges with positive used flow from:

# source -> sink
# and extracts paths like:

# start -> A -> B -> goal
# In short:

# Min-Cost Max-Flow result = flow spread across edges
# decompose_paths = convert that flow into list of source-to-sink paths
# Those paths become the path bank used later by your scheduler.