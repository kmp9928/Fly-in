from typing import List, Dict, Tuple
from errors import GraphError
from models import DronesN, Node, Connection


type ConnectingNodes = Tuple[str, str]


class Graph:
    """Represents the operational network topology for routing drones.

    Attributes:
        start: The designated origin hub.
        end: The designated destination hub.
        nodes: A list of intermediate hubs within the network.
        connections: A list of bidirectional routing links.
        drones: The global drone configuration wrapper.
    """
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
        """Initializes the network graph with hubs and routing paths."""
        self.start = start
        self.end = end
        self.nodes = nodes
        self.connections = connections
        self.drones = drones

    def get_all_nodes(self) -> Dict[str, Node]:
        """Returns a mapping of all network hub names (start, intermediate
            and end) to their objects."""
        all_nodes: List[Node] = [self.start] + self.nodes + [self.end]
        return {node.name: node for node in all_nodes}

    def get_node(self, name: str) -> Node:
        """Retrieves a specific node by its unique string name.

        Raises:
            GraphError: If no node matching the name exists in the network.
        """
        if name == self.start.name:
            return self.start
        if name == self.end.name:
            return self.end

        for node in self.nodes:
            if node.name == name:
                return node

        raise GraphError(f"Node '{name}' not found in the network.")

    def get_node_edges(self, node: str) -> List[Node]:
        """Retrieves all target neighbor nodes reachable directly from the
            specified origin node."""
        return [
            self.get_node(connection.to_hub)
            for connection in self.connections if connection.from_hub == node
        ]

    def get_all_edges(self) -> Dict[ConnectingNodes, int]:
        """Maps directional connection pairs to their maximum link capacity."""
        return {
            (edge.from_hub, edge.to_hub): edge.max_link_capacity
            for edge in self.connections
        }

    def get_edge(self, edge: ConnectingNodes) -> int:
        """Retrieves the link capacity for a node pair, checking both
            directions if necessary."""
        try:
            return self.get_all_edges()[edge]
        except KeyError:
            return self.get_all_edges()[(edge[1], edge[0])]

    def get_start(self) -> Node:
        """Returns the network's start node."""
        return self.start

    def get_end(self) -> Node:
        """Returns the network's end node."""
        return self.end

    def get_drones_n(self) -> int:
        """Returns the configured total capacity number of drones."""
        return self.drones.number


# if __name__ == "__main__":
#     network = NetworkParser.load("01_linear_path.txt")
#     graph = Graph(network)
#     print(graph.get_graph())
#     print(graph.get_nodes())


#get_graph retuns list without any issues since the initial construction was with a set so no duplicates
