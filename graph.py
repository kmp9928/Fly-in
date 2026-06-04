from typing import List, Dict
from models import Zone, Connection


class Graph:
    # graph: Dict[str, Set[str]]
    start: Zone
    end: Zone
    nodes: List[Zone]
    connections: List[Connection]
    drones: int
    # all_nodes: List[Zone]

    def __init__(
        self,
        start: Zone,
        end: Zone,
        nodes: List[Zone],
        connections: List[Connection],
        drones: int
    ) -> None:
        # self.graph = {}
        self.start = start
        self.end = end
        self.nodes = nodes
        self.connections = connections
        self.drones = drones
        # all_hubs = [network.start_hub] + network.hubs + [network.end_hub]
        # hubs_names = [hub.name for hub in all_hubs]


        # for hub in all_hubs:
        #     key = hub.name
        #     self.graph[key] = set()
        #     self.nodes[key] = hub
        #     for connection in self.connections:
        #         if key in [connection.from_hub, connection.to_hub]:
        #             match = (
        #                 connection.from_hub
        #                 if connection.from_hub != key else
        #                 connection.to_hub
        #             )
        #             self.graph[key].add(all_hubs[hubs_names.index(match)].name)


    def get_nodes(self) -> Dict[str, Zone]:
        all_nodes: List[Zone] = [self.start] + self.nodes + [self.end]
        return {node.name: node for node in all_nodes}

    def get_edges(self, node: str) -> List[Zone]:
        return [
            self.get_nodes()[connection.to_hub]
            for connection in self.connections if connection.from_hub == node
        ]

    # def get_graph(self) -> Dict[str, List[str]]:
    #     return {
    #         node: [edge for edge in edges]
    #         for node, edges in self.graph.items()
    #     }

    # def get_connections(self) -> List[Connection]:
    #     return self.connections


# if __name__ == "__main__":
#     network = NetworkParser.load("01_linear_path.txt")
#     graph = Graph(network)
#     print(graph.get_graph())
#     print(graph.get_nodes())


#get_graph retuns list without any issues since the initial construction was with a set so no duplicates
