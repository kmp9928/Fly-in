import plotly.graph_objects as go
from typing import Dict
from graph import Graph


class Renderer:
    @staticmethod
    def render_network(graph: Graph) -> None:
        edge_x = []
        edge_y = []

        for node in graph.get_all_nodes().values():
            x0 = node.x
            y0 = node.y
            for edge in graph.get_node_edges(node.name):
                x1 = edge.x
                y1 = edge.y
                edge_x.extend([x0, x1, None])
                edge_y.extend([y0, y1, None])

        edge_trace = go.Scatter(
            x=edge_x, y=edge_y,
            line=dict(width=2, color='gray'),
            mode='lines'
        )

        node_x = [node.x for node in graph.get_all_nodes().values()]
        node_y = [node.y for node in graph.get_all_nodes().values()]
        node_name = [node for node in graph.get_all_nodes().keys()]
        node_color = [
            Renderer.map_color(node.color.value)
            for node in graph.get_all_nodes().values()
        ]

        node_trace = go.Scatter(
            x=node_x, y=node_y,
            mode='markers+text',
            text=node_name, textposition="top center",
            marker=dict(
                size=25, color=node_color,
                line=dict(width=2, color='black')
            )
        )

        figure = go.Figure(
            data=[edge_trace, node_trace],
            layout=go.Layout(
                showlegend=False,
                xaxis=dict(
                    showgrid=False, zeroline=False, showticklabels=False
                ),
                yaxis=dict(
                    showgrid=False, zeroline=False, showticklabels=False
                )
            )
        )
        figure.show()

    @staticmethod
    def map_color(color: str) -> str:
        colors: Dict[str, str] = {
            "green": "#008000",
            "blue": "#0000FF",
            "red": "#FF0000",
            "yellow": "#FFFF00",
            "orange": "#FFA500"
        }
        return colors[color]

    # def render_drones(self) -> None: #adds drones on top of graph


# if __name__ == "__main__":
#     network = NetworkParser.load("03_priority_puzzle.txt")
#     graph = network.to_graph()
#     Renderer().render_network(graph)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# import plotly.graph_objects as go

# # 1. YOUR CUSTOM DATA STRUCTURES (No graph libraries allowed!)
# node_positions = {"Start": (0, 5), "Alpha": (3, 8), "Beta": (3, 2), "End": (6, 5)}
# g/network = {"Start": ["Alpha", "Beta"], "Alpha": ["End"], "Beta": ["End"], "End": []}
# drones = [{"id": "Drone_1", "at_node": "Alpha"}, {"id": "Drone_2", "at_node": "Start"}]

# fig = go.Figure()

# # 2. TRACE 1: Draw the connections (Lines)
# edge_x = []
# edge_y = []
# for current_node, neighbors in network.items():
#     x1, y1 = node_positions[current_node]
#     for neighbor in neighbors:
#         x2, y2 = node_positions[neighbor]
#         edge_x.extend([x1, x2, None]) # 'None' creates a clean break between line segments
#         edge_y.extend([y1, y2, None])

# fig.add_trace(go.Scatter(
#     x=edge_x, y=edge_y, 
#     line=dict(width=2, color='gray', dash='dash'), 
#     mode='lines', name='Paths'
# ))

# # 3. TRACE 2: Draw the waypoints (Nodes)
# node_x = [pos[0] for pos in node_positions.values()]
# node_y = [pos[1] for pos in node_positions.values()]
# node_names = list(node_positions.keys())

# fig.add_trace(go.Scatter(
#     x=node_x, y=node_y, 
#     mode='markers+text', 
#     text=node_names, textposition="top center",
#     marker=dict(size=25, color='lightblue', line=dict(width=2, color='black')),
#     name='Waypoints'
# ))

# # 4. TRACE 3: Draw the drones exactly on top of their current coordinates
# drone_x = [node_positions[d["at_node"]][0] for d in drones]
# drone_y = [node_positions[d["at_node"]][1] for d in drones]
# drone_ids = [d["id"] for d in drones]

# fig.add_trace(go.Scatter(
#     x=drone_x, y=drone_y, 
#     mode='markers+text', 
#     text=drone_ids, textposition="bottom center",
#     marker=dict(size=15, color='red', symbol='triangle-up'),
#     name='Active Drones'
# ))

# # Layout configurations
# fig.update_layout(
#     title="Custom Drone Network (Plotly Stack)",
#     showlegend=True,
#     xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
#     yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
#     plot_bgcolor='white'
# )

# fig.show()
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -