from typing import Dict, List, Tuple
from input_parser import Zone, Connection, NetworkParser
from graph import Graph


class Edge:
    def __init__(
            self, to: int, capacity: int, cost: int, reverse=0, original=False
    ) -> None:
        self.to = to
        self.capacity = capacity
        self.cost = cost
        self.reverse = reverse
        self.original = original

    # def __repr__(self) -> str:
    #     return (
    #         f"Edge(to={self.to!r}, "
    #         f"capacity={self.capacity}, "
    #         f"cost={self.cost}, "
    #         f"reverse={self.reverse},"
    #         f"original={self.original})"
    #     )


class FlowGraph:
    flow_graph: Dict[str, List[Edge]]

    def __init__(
            self, connections: List[Connection], nodes: Dict[str, Zone]
    ) -> None:
        self.flow_graph = {}
        start_in_goal_out: List[str] = [
            list(nodes.keys())[0] + "_in", list(nodes.keys())[-1] + "_out"
        ]

        for connection in connections:
            self.add_edge(connection, start_in_goal_out)

        for node in list(nodes.values())[1:-1]:
            self.add_node_edge(node)

        for node_id, edges in self.flow_graph.items():
            for edge in edges:
                for i, edge_r in enumerate(self.flow_graph[edge.to], start=0):
                    if edge_r.to == node_id:
                        break
                edge.reverse = i

    def add_edge(self, edge: Connection, start_in_goal_out: List[str]) -> None:
        combinations: List[Tuple[str, str]] = [
            (edge.from_hub, edge.to_hub), (edge.to_hub, edge.from_hub)
        ]

        for combination in combinations:
            forward = Edge(
                to=combination[1] + "_in",
                capacity=edge.max_link_capacity,
                cost=1,
                original=True
            )
            node_id = combination[0] + "_out"
            if (
                node_id not in start_in_goal_out and
                forward.to not in start_in_goal_out
            ):
                if self.flow_graph.get(node_id) is None:
                    self.flow_graph[node_id] = [forward]
                else:
                    self.flow_graph[node_id].extend([forward])

            reverse = Edge(
                to=combination[1] + "_out",
                capacity=0,
                cost=-1,
            )
            node_id = combination[0] + "_in"
            if (
                node_id not in start_in_goal_out and
                reverse.to not in start_in_goal_out
            ):
                if self.flow_graph.get(node_id) is None:
                    self.flow_graph[node_id] = [reverse]
                else:
                    self.flow_graph[node_id].extend([reverse])

    def add_node_edge(self, node: Zone) -> None:
        node_costs: Dict[str, int] = {
            "normal": 1,
            "restricted": 10,
            "priority": 0
        }

        if node.zone.value != "blocked":
            self.flow_graph.get(node.name + "_out").extend([
                Edge(
                    to=node.name + "_in",
                    capacity=0,
                    cost=-node_costs[node.zone.value],
                )
            ])
            self.flow_graph.get(node.name + "_in").extend([
                Edge(
                    to=node.name + "_out",
                    capacity=node.max_drones,
                    cost=node_costs[node.zone.value],
                    original=True
                )
            ])

    def is_forbidden(self, node: str) -> bool:
        if 

if __name__ == "__main__":
    network = NetworkParser.load("03_priority_puzzle.txt")
    graph = Graph.to_graph(network)
    # Renderer().render_network(graph)
    flow_graph = FlowGraph(graph)