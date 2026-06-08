from typing import List, Dict, Tuple
from models import DronesN, Node, Connection


type ConnectingNodes = Tuple[str, str]


class Graph:
    start: Node
    end: Node
    nodes: List[Node]
    connections: List[Connection]
    drones: DronesN

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
        self.drones = drones

    def get_all_nodes(self) -> Dict[str, Node]:
        all_nodes: List[Node] = [self.start] + self.nodes + [self.end]
        return {node.name: node for node in all_nodes}

    def get_node(self, name: str) -> Node:
        if name == self.start.name:
            return self.start
        if name == self.end.name:
            return self.end

        for node in self.nodes:
            if node.name == name:
                return node

    def get_node_edges(self, node: str) -> List[Node]:
        return [
            self.get_node(connection.to_hub)
            for connection in self.connections if connection.from_hub == node
        ]

    def get_all_edges(self) -> Dict[ConnectingNodes, int]:
        return {
            (edge.from_hub, edge.to_hub): edge.max_link_capacity
            for edge in self.connections
        }

    def get_edge(self, edge: str) -> int: #ADD IN @ PLACES????!!!
        return self.get_all_edges()[edge]

    def get_start(self) -> Node:
        return self.start

    def get_end(self) -> Node:
        return self.end

    def get_drones_n(self) -> int:
        return self.drones.number


# if __name__ == "__main__":
#     network = NetworkParser.load("01_linear_path.txt")
#     graph = Graph(network)
#     print(graph.get_graph())
#     print(graph.get_nodes())


#get_graph retuns list without any issues since the initial construction was with a set so no duplicates
