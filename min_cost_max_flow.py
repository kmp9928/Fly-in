from sys import maxsize
from dataclasses import dataclass
from typing import List, Dict, Tuple
from errors import FlowGraphError
from models import Node
from graph import Graph, ConnectingNodes


@dataclass
class Edge:
    """Represents a directional network routing vector within the flow network.

    Attributes:
        to: The identifier string of the target sub-node.
        capacity: The total drone throughput limit for this edge.
        cost: The routing weight associated with parsing this edge.
        reverse: The index position of this edge's matching residual vector
            in the target node's edge array.
        original: True if this represents a physical connection path rather
            than an artificial residual edge.
    """
    to: str
    capacity: int
    cost: int
    reverse: int = 0
    original: bool = False


class FlowGraph:
    """Transforms a standard topology map into a residual network for
    Minimum-Cost Maximum-Flow algorithm.

    Splits structural nodes into independent `_in` and `_out` variations to
    manage vertex-level capacities, establishing valid edge pairs, tracking
    flow limits, and configuring structural blocks.

    Attributes:
        start: The initial source identifier string.
        end: The target sink identifier string.
        flow_graph: Adjacency list mapping node names to of reachable Edges.
        forbidden: List of sub-nodes restricted from standard traffic loops.
    """
    start: str
    end: str
    flow_graph: Dict[str, List[Edge]]
    forbidden: List[str]

    def __init__(self, graph: Graph) -> None:
        """Initializes and expands a base network topology into a strict
        network flow graph."""
        self.start = FlowGraph.make_out_name(graph.get_start().name)
        self.end = FlowGraph.make_in_name(graph.get_end().name)
        self.flow_graph = {}
        self.forbidden = [
            FlowGraph.make_in_name(graph.start.name),
            FlowGraph.make_out_name(graph.end.name)
        ]

        for connection, capacity in graph.get_all_edges().items():
            from_node: str = graph.get_node(connection[0]).zone.value
            to_node: str = graph.get_node(connection[1]).zone.value
            edge_cost: int = (
                2 if from_node == "priority" and to_node == "priority"
                else 1
            )
            self.add_edge_pair(connection, capacity, edge_cost)
            self.add_residual_pair(connection, edge_cost)

        for node in graph.get_all_nodes().values():
            if node.zone.value != "blocked":
                self.add_inner_pair(node)

        self.add_reverse_index()

    @staticmethod
    def make_in_name(name: str) -> str:
        """Generates the standardized identifier for a node's incoming
        interface sub-node."""
        return name + "_in"

    @staticmethod
    def make_out_name(name: str) -> str:
        """Generates the standardized identifier for a node's outgoing
        interface sub-node."""
        return name + "_out"

    def add_edge_pair(
        self, edges: ConnectingNodes, capacity: int, edge_cost: int
    ) -> None:
        """Establishes bidirectional baseline flow paths between two connected
        network nodes."""
        self.add_edge(
            FlowGraph.make_out_name(edges[0]),
            FlowGraph.make_in_name(edges[1]),
            capacity,
            edge_cost,
            True
        )
        self.add_edge(
            FlowGraph.make_out_name(edges[1]),
            FlowGraph.make_in_name(edges[0]),
            capacity,
            edge_cost,
            True
        )

    def add_residual_pair(
        self, edges: ConnectingNodes, edge_cost: int
    ) -> None:
        """Constructs backward residual capacities across external links to
        facilitate flow cancellation."""
        self.add_edge(
            FlowGraph.make_in_name(edges[0]),
            FlowGraph.make_out_name(edges[1]),
            0,
            -edge_cost
        )
        self.add_edge(
            FlowGraph.make_in_name(edges[1]),
            FlowGraph.make_out_name(edges[0]),
            0,
            -edge_cost
        )

    def add_inner_pair(self, node: Node) -> None:
        """Links a node's internal 'in' and 'out' sub-nodes, enforcing regional
        pricing policies."""
        node_costs: Dict[str, int] = {
            "normal": 1,
            "restricted": 10,
            "priority": -2
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
        """Evaluates whether a sub-node is restricted from entering or leaving
        the network loop."""
        return node_name in self.forbidden

    def add_edge(
        self,
        from_id: str,
        to_id: str,
        capacity: int,
        cost: int,
        original: bool = False
    ) -> None:
        """Registers a direction-specific edge onto the graph layout if
        boundaries are structurally valid."""
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
        """Populates cross-reference indices to sync back-edges instantly
        during min-cost max-flow mutations."""
        for from_id, edges in self.flow_graph.items():
            for edge in edges:
                for i, edges_of_reverse in enumerate(self.flow_graph[edge.to]):
                    if edges_of_reverse.to == from_id:
                        break
                edge.reverse = i

    def get_nodes(self) -> List[str]:
        """Retrieves a list of all active registered tracking sub-nodes."""
        return list(self.flow_graph.keys())

    def get_edges(self, node: str) -> List[Edge]:
        """Retrieves the list of edges mapped to a given sub-node."""
        return self.flow_graph[node]

    def get_start(self) -> str:
        """Returns the name of the structural network source."""
        return self.start

    def get_end(self) -> str:
        """Returns the name of the structural network sink."""
        return self.end


type Path = List[str]


class MinCostMaxFlowAlgorithm:
    """Implements a Minimum-Cost Maximum-Flow optimizer to discover
    congestion-aware drone routes.

    Employs the Bellman-Ford algorithm iteratively to find the cheapest paths
    through an expanded residual flow network, augmenting capacity dynamically
    until maximum network traffic is reached and collapses the artificial node
    interfaces back into concrete paths.
    """
    def find_paths(self, graph: Graph) -> List[Path]:
        """Calculates a set of optimal, capacity-respecting paths across the
        network topology.

        Transforms the source structural layout into a specialized FlowGraph,
        repeatedly executes shortest-path calculations, increases bottle-neck
        allocations and handles final pathway extraction.

        Args:
            graph: The foundational structure housing nodes and connections.

        Returns:
            A list of path sequences containing structural node identifiers.

        Raises:
            FlowGraphError: If the end is disconnected from the network source.
        """
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

    @staticmethod
    def bellman_ford(
        flow_graph: FlowGraph
    ) -> Tuple[Dict[str, Tuple[str | None, int]], int] | None:
        """Executes a Bellman-Ford traversal to find the cheapest path with
        remaining capacity.

        Tracks node-to-node routing values while identifying edge capacity
        constraints, mapping out predecessor linkages and auditing structural
        systems against infinite negative loops.

        Args:
            flow_graph: The active residual network representation.

        Returns:
            A tuple containing:
                - A dictionary tracking predecessor trace mappings.
                - The accumulated minimum distance value to the end node.
                - Returns None if no path to the end node can be located.

        Raises:
            FlowGraphError: If a structural negative weight loop is detected.
        """
        nodes: List[str] = flow_graph.get_nodes()
        distances: Dict[str, int] = {
            node: maxsize for node in nodes
        }
        predecessors: Dict[str, Tuple[str | None, int]] = {
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

    @staticmethod
    def augment(
        flow_graph: FlowGraph, predecessors: Dict[str, Tuple[str | None, int]]
    ) -> int:
        """Pushes units of traffic along a discovered trace path and adjusts
        capacities.

        Traces the given path backward to find the strict bottleneck limit,
        deducts that amount from forward edge capacities and adds it back to
        the corresponding residual reverse links.

        Args:
            flow_graph: The active residual network layout.
            predecessors: The trace path generated by the pathfinder.

        Returns:
            The total volume of units successfully added along the target path.
        """
        amount: int = maxsize
        node: str = flow_graph.get_end()

        while node != flow_graph.get_start():
            previous, i = predecessors[node]
            assert previous is not None
            edge = flow_graph.get_edges(previous)[i]
            amount = min(amount, edge.capacity)
            node = previous

        node = flow_graph.get_end()
        while node != flow_graph.get_start():
            previous, i = predecessors[node]
            assert previous is not None
            edge = flow_graph.get_edges(previous)[i]
            reverse = flow_graph.get_edges(edge.to)[edge.reverse]
            edge.capacity -= amount
            reverse.capacity += amount
            node = previous

        return amount

    @staticmethod
    def decompose_paths(flow_graph: FlowGraph) -> List[Path]:
        """Breaks down a global network flow layout into individual sequence
        arrays.

        Scans structural links to measure capacities, traces matching pathways
        from source to sink and strips out excess traffic values
        stage-by-stage.

        Args:
            flow_graph: The complete optimized residual network state.

        Returns:
            A list of paths using temporary `_in` and `_out` interfaces.

        Raises:
            FlowGraphError: If cyclical looping occurs or if a trace stops
                prematurely.
        """
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
        """Cleans up internal flow sub-nodes back into recognizable labels.

        Removes structural string markers like `_in` and `_out` from step
        trackers and filters out consecutive duplicate values caused by
        structural node-splitting.

        Args:
            paths: A list of raw sub-node routing sequences.

        Returns:
            A simplified list of concrete original map node sequences.
        """
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
            if new_path not in result:
                result.append(new_path)

        return result
