from typing import List, Dict, Tuple
from models import DronesN, Node, Connection


class Graph:
    start: Node
    end: Node
    nodes: List[Node]
    connections: List[Connection]
    drones: int

    def __init__(
        self,
        start: Node,
        end: Node,
        nodes: List[Node],
        connections: List[Connection],
        drones: DronesN
    ) -> None:
        self.start = start
        self.end = end
        self.nodes = nodes
        self.connections = connections
        self.drones = drones.number

    def get_all_nodes(self) -> Dict[str, Node]:
        all_nodes: List[Node] = [self.start] + self.nodes + [self.end]
        return {node.name: node for node in all_nodes}

    def get_node_edges(self, node: str) -> List[Node]:
        return [
            self.get_all_nodes()[connection.to_hub]
            for connection in self.connections if connection.from_hub == node
        ]

    def get_all_edges(self) -> Dict[Tuple[str, str], int]:
        return {
            (edge.from_hub, edge.to_hub): edge.max_link_capacity
            for edge in self.connections
        }


# if __name__ == "__main__":
#     network = NetworkParser.load("01_linear_path.txt")
#     graph = Graph(network)
#     print(graph.get_graph())
#     print(graph.get_nodes())


#get_graph retuns list without any issues since the initial construction was with a set so no duplicates
