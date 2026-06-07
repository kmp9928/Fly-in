from typing import Dict, List, Tuple
from input_parser import Node
from graph import Graph
from dataclasses import dataclass


@dataclass
class Edge:
    to: str
    capacity: int
    cost: int
    reverse: int = 0
    original: bool = False

    # def __repr__(self) -> str:
    #     return (
    #         f"Edge(to={self.to!r}, "
    #         f"capacity={self.capacity}, "
    #         f"cost={self.cost}, "
    #         f"reverse={self.reverse},"
    #         f"original={self.original})"
    #     )


class FlowGraph:
    start: str
    end: str
    nodes: Dict[str, Node]
    connections: Dict[Tuple[str, str], int]
    flow_graph: Dict[str, List[Edge]]
    forbidden: List[str]

    def __init__(self, graph: Graph) -> None:
        self.start = FlowGraph.make_out_name(graph.start.name)
        self.end = FlowGraph.make_in_name(graph.end.name)
        self.nodes = graph.get_all_nodes()
        self.connections = graph.get_all_edges()
        self.flow_graph = {}
        self.forbidden = [
            FlowGraph.make_in_name(graph.start.name),
            FlowGraph.make_out_name(graph.end.name)
        ]

        for connection, capacity in self.connections.items():
            self.add_edge_pair(connection, capacity)
            self.add_residual_pair(connection)

        for node in self.nodes.values():
            if node.zone.value != "blocked":
                self.add_inner_pair(node)

        self.add_reverse_index()

    @staticmethod
    def make_in_name(name: str) -> str:
        return name + "_in"

    @staticmethod
    def make_out_name(name: str) -> str:
        return name + "_out"

    def add_edge_pair(self, edges: Tuple[str, str], capacity: int) -> None:
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

    def add_residual_pair(self, edges: Tuple[str, str]) -> None:
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


# if __name__ == "__main__":
#     network = NetworkParser.load("01_linear_path.txt")
#     graph = network.to_graph()
#     flow_graph = FlowGraph(graph)
#     for node, edges in FlowGraph(graph).flow_graph.items():
#         print(f"node {node}: {repr(edges)}")

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Residual reverse edge

# If you add the real tunnel edge:

# start_out -> A_in
# then Min-Cost Max-Flow automatically creates the reverse residual edge:

# A_in -> start_out
# with:

# capacity = 0
# cost = -1
# original = False
# This is needed for residual flow undo/rerouting.

# So in this case:

# A_in -> start_out
# exists because of the residual graph, not because the tunnel is bidirectional.


# For the real graph:

# start -- A -- B -- end
# assuming tunnels are bidirectional, the tunnel A-B gives you two real/original tunnel edges:

# A_out -> B_in   original=True
# B_out -> A_in   original=True
# Because drones are allowed to move:

# A -> B
# and also:

# B -> A
# Then each real/original tunnel edge gets a residual reverse edge:

# B_in -> A_out   original=False
# A_in -> B_out   original=False
# So all four are:

# A_out -> B_in   real tunnel A to B
# B_in -> A_out   residual undo of A to B

# B_out -> A_in   real tunnel B to A
# A_in -> B_out   residual undo of B to A
# You are right that the last two may feel strange in a simple path from start to end:

# B_out -> A_in
# A_in -> B_out
# But if the map tunnel is bidirectional, B_out -> A_in does exist as the real reverse tunnel direction.

# A_in -> B_out does not exist as a real tunnel. It exists only as the residual undo edge for B_out -> A_in.

# If you decide not to model bidirectional movement, then for a directed path:

# start -> A -> B -> end
# you would only add:

# A_out -> B_in   original=True
# B_in -> A_out   original=False
# and not:

# B_out -> A_in
# A_in -> B_out
# But since the subject says tunnels are bidirectional, the four-edge set is correct.

# The key distinction:

# *_out -> neighbor_in = real tunnel direction
# neighbor_in -> *_out = residual undo edge
# So B_out -> A_in is real if B -> A is allowed.
# A_in -> B_out is residual, not real.

    # def add_edge(self, edge: Connection) -> None:
    #     combinations: List[Tuple[str, str]] = [
    #         (edge.from_hub, edge.to_hub), (edge.to_hub, edge.from_hub)
    #     ]

    #     for combination in combinations:
    #         forward = Edge(
    #             to=combination[1] + "_in",
    #             capacity=edge.max_link_capacity,
    #             cost=1,
    #             original=True
    #         )
    #         node_id = combination[0] + "_out"
    #         if (
    #             not self.is_forbidden(node_id) and
    #             not self.is_forbidden(forward.to)
    #         ):
    #             if self.flow_graph.get(node_id) is None:
    #                 self.flow_graph[node_id] = [forward]
    #             else:
    #                 self.flow_graph[node_id].extend([forward])

    #         reverse = Edge(
    #             to=combination[1] + "_out",
    #             capacity=0,
    #             cost=-1,
    #         )
    #         node_id = combination[0] + "_in"
    #         if (
    #             not self.is_forbidden(node_id) and
    #             not self.is_forbidden(reverse.to)
    #         ):
    #             if self.flow_graph.get(node_id) is None:
    #                 self.flow_graph[node_id] = [reverse]
    #             else:
    #                 self.flow_graph[node_id].extend([reverse])

    # def add_node_edge(self, node: Zone) -> None:
    #     node_costs: Dict[str, int] = {
    #         "normal": 1,
    #         "restricted": 10,
    #         "priority": 0
    #     }

    #     if node.zone.value != "blocked" and not self.is_forbidden(node.name):
    #         self.flow_graph.get(node.name + "_out").extend([
    #             Edge(
    #                 to=node.name + "_in",
    #                 capacity=0,
    #                 cost=-node_costs[node.zone.value],
    #             )
    #         ])
    #         self.flow_graph.get(node.name + "_in").extend([
    #             Edge(
    #                 to=node.name + "_out",
    #                 capacity=node.max_drones,
    #                 cost=node_costs[node.zone.value],
    #                 original=True
    #             )
    #         ])
