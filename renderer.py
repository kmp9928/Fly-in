import plotly.graph_objects as go
from typing import Dict, List
import math
from models import Node
from graph import Graph
from route_planner import Drone


class Renderer:
    @staticmethod
    def render_simulation(
        graph: Graph, simulation_history: Dict[int, Dict[str, List[Drone]]]
    ) -> None:
        all_nodes: Dict[str, Node] = graph.get_all_nodes()
        nodes_n: int = len(all_nodes)
        dynamic_node_size: int = max(12, min(50, 90 / (nodes_n ** 0.5)))
        dynamic_drone_size: int = max(8, dynamic_node_size * 0.50)
        dynamic_font_size: int = max(8, min(14, 40 / (nodes_n ** 0.3)))

        edge_x = []
        edge_y = []

        for node in all_nodes.values():
            x0 = node.x
            y0 = node.y
            for edge in graph.get_node_edges(node.name):
                x1 = edge.x
                y1 = edge.y
                edge_x.extend([x0, x1, None])
                edge_y.extend([y0, y1, None])

        edge_trace = go.Scatter(
            x=edge_x,
            y=edge_y,
            line=dict(width=max(1, dynamic_node_size * 0.08), color='gray'),
            mode='lines'
        )

        node_x = [node.x for node in all_nodes.values()]
        node_y = [node.y for node in all_nodes.values()]
        node_name = list(all_nodes.keys())
        node_color = []
        node_symbol = []
        line_color = []
        line_width = []
        for node in all_nodes.values():
            if node.color.value == "rainbow":
                node_color.append(Renderer.map_color("magenta"))
                node_symbol.append('circle')
                line_color.append(Renderer.map_color("teal"))
                line_width.append(max(2, dynamic_node_size * 0.12))
            else:
                node_color.append(Renderer.map_color(node.color.value))
                node_symbol.append('circle')
                line_color.append('rgba(0,0,0,0)')
                line_width.append(0)

        node_trace = go.Scatter(
            x=node_x,
            y=node_y,
            mode='markers+text',
            text=node_name,
            textposition="top center",
            textfont=dict(size=dynamic_font_size),
            marker=dict(
                size=dynamic_node_size,
                color=node_color,
                symbol=node_symbol,
                line=dict(color=line_color, width=line_width)
            )
        )

        init_drone_x = []
        init_drone_y = []
        init_drone_ids = []
        initial_turn = simulation_history[0] if simulation_history else []
        node_occupancy_count: Dict[str, int] = {
            point: 0 for point in initial_turn.keys()
        }
        for point, drone_list in initial_turn.items():
            radius = (dynamic_node_size / 150)

            for drone in drone_list:
                count: int = node_occupancy_count[point]

                if count == 0:
                    final_x = drone.x
                    final_y = drone.y
                else:
                    angle = count * (2 * math.pi / 5)
                    final_x = drone.x + radius * math.cos(angle)
                    final_y = drone.y + radius * math.sin(angle)

                init_drone_x.append(final_x)
                init_drone_y.append(final_y)
                init_drone_ids.append(drone.id)

                node_occupancy_count[point] += 1

        drone_trace = go.Scatter(
            x=init_drone_x,
            y=init_drone_y,
            mode='markers+text',
            text=init_drone_ids,
            ids=init_drone_ids,
            textposition="bottom center",
            textfont=dict(size=dynamic_font_size * 1.1, color='black'),
            marker=dict(
                size=dynamic_drone_size,
                color='black',
                symbol='triangle-up'
            ),
            name='Drones'
        )

        frames = []
        for turn, drone_data_per_turn in simulation_history.items():
            dx = []
            dy = []
            d_id = []
            node_occupancy_count: Dict[str, int] = {
                point: 0 for point in drone_data_per_turn.keys()
            }
            radius = (dynamic_node_size / 150)

            for point, drone_list in drone_data_per_turn.items():
                for drone in drone_list:
                    count: int = node_occupancy_count[point]

                    if count == 0:
                        final_x = drone.x
                        final_y = drone.y
                    else:
                        angle = count * (2 * math.pi / 5)
                        final_x = drone.x + radius * math.cos(angle)
                        final_y = drone.y + radius * math.sin(angle)

                    dx.append(final_x)
                    dy.append(final_y)
                    d_id.append(drone.id)

                    node_occupancy_count[point] += 1

            frames.append(
                go.Frame(
                    data=[
                        go.Scatter(
                            x=dx,
                            y=dy,
                            text=d_id,
                            ids=d_id,
                            mode='markers+text',
                            textfont=dict(size=dynamic_font_size * 1.1),
                            marker=dict(
                                size=dynamic_drone_size,
                                color='black',
                                symbol='triangle-up'
                            )
                        )
                    ],
                    traces=[2],
                    layout=go.Layout(
                        title=dict(text=f"Drone Simulation - Turn {turn}")
                    )
                )
            )

        figure = go.Figure(
            data=[edge_trace, node_trace, drone_trace],
            layout=go.Layout(
                showlegend=False,
                xaxis=dict(
                    showgrid=False, zeroline=False, showticklabels=False
                ),
                yaxis=dict(
                    showgrid=False,
                    zeroline=False,
                    showticklabels=False,
                    scaleanchor="x",
                    scaleratio=1
                ),
                updatemenus=[{
                    "type": "buttons",
                    "buttons": [
                        {
                            "label": "Play", 
                            "method": "animate", 
                            "args": [
                                None,
                                {
                                    "frame": {
                                        "duration": 2000, "redraw": False
                                    },
                                    "transition": {"duration": 1000},
                                    "fromcurrent": True
                                }
                            ]
                        },
                        {
                            "label": "Pause",
                            "method": "animate",
                            "args": [
                                [None],
                                {
                                    "frame": {"duration": 0, "redraw": False},
                                    "mode": "immediate"
                                }
                            ]
                        }
                    ]
                }]
            ),
            frames=frames
        )
        figure.show()

    @staticmethod
    def map_color(color: str) -> str:
        colors: Dict[str, str] = {
            "black": "#000000",
            "blue": "#0000FF",
            "brown": "#964B00",
            "crimson": "#DC143C",
            "darkred": "#8B0000",
            "gold": "#FFD700",
            "green": "#008000",
            "magenta": "#FF007F",
            "maroon": "#800000",
            "orange": "#FFA500",
            "purple": "#800080",
            "red": "#FF0000",
            "teal": "#00F5D4",
            "violet": "#7F00FF",
            "white": "#FFFFFF",
            "yellow": "#FFFF00"
        }
        return colors[color]


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