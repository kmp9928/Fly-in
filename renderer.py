import plotly.graph_objects as go  # type: ignore
from typing import Dict, List
import math
from models import Node
from graph import Graph
from route_planner import Drone


class Renderer:
    """Handles the graphical visualization and animation of the drone routing
    system."""
    @staticmethod
    def render_simulation(
        graph: Graph, simulation_history: Dict[int, Dict[str, List[Drone]]]
    ) -> None:
        """Generates an animated Plotly canvas representing the complete
        network simulation.

        Calculates dynamic asset scaling based on topological density, renders
        underlying connection edges and custom hub markers, and maps orbital
        paths to prevent overlapping drone icons at identical nodes across step
        frames.

        Args:
            graph: The foundational structure housing nodes and connections.
            simulation_history: Chronological dictionary mapping turn sequences
                to hub keys containing active drone assets.
        """
        all_nodes: Dict[str, Node] = graph.get_all_nodes()
        nodes_n: int = len(all_nodes)
        dynamic_node_size: float = max(12, min(50, 90 / (nodes_n ** 0.5)))
        dynamic_drone_size: float = max(8, dynamic_node_size * 0.50)
        dynamic_font_size: float = max(8, min(14, 40 / (nodes_n ** 0.3)))

        edge_x: List[int | None] = []
        edge_y: List[int | None] = []

        for node in all_nodes.values():
            x0: int = node.x
            y0: int = node.y
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

        node_x: List[int] = [node.x for node in all_nodes.values()]
        node_y: List[int] = [node.y for node in all_nodes.values()]
        node_name: List[str] = list(all_nodes.keys())
        node_color: List[str] = []
        node_symbol: List[str] = []
        line_color: List[str] = []
        line_width: list[float] = []
        for node in all_nodes.values():
            if node.color.value == "rainbow":
                node_color.append(Renderer.map_color("magenta"))
                node_symbol.append('circle')
                line_color.append(Renderer.map_color("teal"))
                line_width.append(max(2, dynamic_node_size * 0.12))
            elif node.color.value == "none":
                node_color.append(Renderer.map_color(node.color.value))
                node_symbol.append('circle')
                line_color.append(Renderer.map_color("black"))
                line_width.append(max(2, dynamic_node_size * 0.12))
            else:
                node_color.append(Renderer.map_color(node.color.value))
                node_symbol.append('circle')
                line_color.append(Renderer.map_color(node.color.value))
                line_width.append(max(2, dynamic_node_size * 0.12))

        node_trace = go.Scatter(
            x=node_x,
            y=node_y,
            mode='markers+text',
            text=node_name,
            textposition="top center",
            textfont=dict(size=dynamic_font_size * 0.6),
            marker=dict(
                size=dynamic_node_size,
                color=node_color,
                symbol=node_symbol,
                line=dict(color=line_color, width=line_width)
            )
        )

        init_drone_x: List[float] = []
        init_drone_y: List[float] = []
        init_drone_ids: List[str] = []
        initial_turn: Dict[str, List[Drone]] = (
            simulation_history[0] if simulation_history else {}
        )
        initial_node_occupancy_count: Dict[str, int] = {
            point: 0 for point in initial_turn.keys()
        }
        for point, drone_list in initial_turn.items():
            radius = (dynamic_node_size / 150)

            for drone in drone_list:
                initial_count: int = initial_node_occupancy_count[point]
                init_x: float
                init_y: float

                if initial_count == 0:
                    init_x = drone.x
                    init_y = drone.y
                else:
                    angle = initial_count * (2 * math.pi / 5)
                    init_x = drone.x + radius * math.cos(angle)
                    init_y = drone.y + radius * math.sin(angle)

                init_drone_x.append(init_x)
                init_drone_y.append(init_y)
                init_drone_ids.append(drone.id)

                initial_node_occupancy_count[point] += 1

        drone_trace = go.Scatter(
            x=init_drone_x,
            y=init_drone_y,
            mode='markers+text',
            text=init_drone_ids,
            ids=init_drone_ids,
            textposition="bottom center",
            textfont=dict(size=dynamic_font_size * 0.6, color='black'),
            marker=dict(
                size=dynamic_drone_size,
                color='black',
                symbol='triangle-up'
            ),
            name='Drones'
        )

        frames = []
        for turn, drone_data_per_turn in simulation_history.items():
            dx: List[float] = []
            dy: List[float] = []
            d_id: List[str] = []
            node_occupancy_count: Dict[str, int] = {
                point: 0 for point in drone_data_per_turn.keys()
            }
            radius = (dynamic_node_size / 150)

            for point, drone_list in drone_data_per_turn.items():
                for drone in drone_list:
                    count: int = node_occupancy_count[point]
                    final_x: float
                    final_y: float

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

            real_turn = min(turn + 1, max(simulation_history.keys()))

            frames.append(
                go.Frame(
                    name=f"turn_{turn}",
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
                        title=dict(text=f"Drone Simulation - Turn {real_turn}")
                    )
                )
            )

        figure = go.Figure(
            data=[edge_trace, node_trace, drone_trace],
            layout=go.Layout(
                title=dict(
                    text="Drone Simulation - Turn 0", x=0.5, xanchor='center'
                ),
                showlegend=False,
                xaxis=dict(
                    showgrid=False,
                    zeroline=False,
                    showticklabels=False,
                    range=[min(node_x) - 1, max(node_x) + 1]
                ),
                yaxis=dict(
                    showgrid=False,
                    zeroline=False,
                    showticklabels=False,
                    scaleanchor="x",
                    scaleratio=1,
                    range=[min(node_y) - 1, max(node_y) + 1]
                ),
                updatemenus=[{
                    "type": "buttons",
                    "buttons": [
                        {
                            "label": "Play",
                            "method": "animate",
                            "args": [
                                [
                                    f"turn_{t}"
                                    for t in simulation_history.keys()
                                ],
                                {
                                    "frame": {
                                        "duration": 2500, "redraw": True
                                    },
                                    "transition": {"duration": 750},
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
        """Maps a human-readable string color identifier to its valid
        Hexadecimal string format.

        Args:
            color: The string name of the target color palette label.

        Returns:
            The matching 6-character hex color code.
        """
        colors: Dict[str, str] = {
            "black": "#000000",
            "blue": "#0000FF",
            "brown": "#964B00",
            "crimson": "#DC143C",
            "cyan": "#00FFFF",
            "darkred": "#8B0000",
            "gold": "#FFD700",
            "green": "#008000",
            "lime": "#32CD32",
            "magenta": "#FF007F",
            "maroon": "#800000",
            "none": "#E5ECF6",
            "orange": "#FFA500",
            "purple": "#800080",
            "red": "#FF0000",
            "teal": "#00F5D4",
            "violet": "#7F00FF",
            "white": "#FFFFFF",
            "yellow": "#FFFF00"
        }
        return colors[color]
