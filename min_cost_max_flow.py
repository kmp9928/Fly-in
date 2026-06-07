from sys import maxsize
from typing import List, Dict, Tuple
from graph import Graph
from flow_graph import FlowGraph
from errors import FlowGraphError


type Path = List[str]


class MinCostMaxFlowAlgorithm:
    def find_paths(self, graph: Graph) -> List[Path]:
        return MinCostMaxFlowTask(graph).run()


class MinCostMaxFlowTask:
    flow_graph: FlowGraph
    start: str
    end: str
    nodes: List[str]

    def __init__(self, graph: Graph) -> None:
        flow_graph = FlowGraph(graph)
        self.flow_graph = flow_graph
        self.start = flow_graph.start
        self.end = flow_graph.end
        self.nodes = flow_graph.get_nodes()

    def run(self) -> List[Path]:
        total_flow: int = 0
        total_cost: int = 0

        while True:
            result = self.bellman_ford()

            if result is None:
                break

            predecessors, path_cost = result
            amount = self.augment(predecessors)

            total_flow += amount
            total_cost += amount * path_cost

        if total_flow == 0:
            raise FlowGraphError("End of network unreachable!")

        return MinCostMaxFlowTask.collapse_paths(self.decompose_paths())

    def bellman_ford(self) -> Tuple[Dict[str, Tuple[str, int]], int]:
        distances: Dict[str, int] = {
            node: maxsize for node in self.nodes
        }
        predecessors: Dict[str, Tuple[str, int]] = {
            node: (None, 0) for node in self.nodes
        }
        distances[self.start] = 0

        for _ in range(len(self.nodes) - 1):
            updated = False

            for node in self.nodes:
                if distances[node] == maxsize:
                    continue

                for i, edge in enumerate(self.flow_graph.get_edges(node)):
                    if edge.capacity <= 0:
                        continue

                    new_distance = distances[node] + edge.cost
                    if new_distance < distances[edge.to]:
                        distances[edge.to] = new_distance
                        predecessors[edge.to] = (node, i)
                        updated = True

            if not updated:
                break

        for node in self.nodes:
            if distances[node] == maxsize:
                continue

            for i, edge in enumerate(self.flow_graph.get_edges(node)):
                if edge.capacity <= 0:
                    continue

                if distances[node] + edge.cost < distances[edge.to]:
                    raise FlowGraphError("Negative cycle detected.")

        if predecessors[self.end][0] is None:
            return None

        return predecessors, distances[self.end]

    def augment(self, predecessors: Dict[str, Tuple[str, int]]) -> int:
        amount: int = maxsize
        node: str = self.end

        while node != self.start:
            previous, i = predecessors[node]
            edge = self.flow_graph.get_edges(previous)[i]
            amount = min(amount, edge.capacity)
            node = previous

        node = self.end
        while node != self.start:
            previous, i = predecessors[node]
            edge = self.flow_graph.get_edges(previous)[i]
            reverse = self.flow_graph.get_edges(edge.to)[edge.reverse]
            edge.capacity -= amount
            reverse.capacity += amount
            node = previous

        return amount

    def decompose_paths(self) -> List[Path]:
        paths: List[Path] = []
        remaining_flow: Dict[Tuple[str, int], int] = {}

        for node in self.nodes:
            for i, edge in enumerate(self.flow_graph.get_edges(node)):
                if edge.original:
                    reverse_edges = self.flow_graph.get_edges(edge.to)
                    used = reverse_edges[edge.reverse].capacity
                    if used > 0:
                        remaining_flow[(node, i)] = used

        while True:
            path = [self.start]
            node = self.start
            visited = set()

            while node != self.end:
                if node in visited:
                    raise FlowGraphError("Cycle while decomposing.")

                visited.add(node)
                next_item = None

                for i, edge in enumerate(self.flow_graph.get_edges(node)):
                    if edge.original and remaining_flow.get((node, i), 0) > 0:
                        next_item = (i, edge)
                        break

                if next_item is None:
                    if node == self.start:
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