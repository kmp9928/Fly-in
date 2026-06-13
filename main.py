from os import listdir
from typing import List
from errors import NetworkFileError, FlowGraphError
from input_parser import NetworkParser
from min_cost_max_flow import MinCostMaxFlowAlgorithm
from route_planner import RoutePlanner
from simulation_engine import SimulationEngine
from renderer import Renderer


class FlyIn:
    """Orchestrates the user interface loops and simulation pipeline for the
    drone routing engine."""
    @staticmethod
    def get_file() -> str:
        """Prompts the user interactively via the CLI to select a target map
        category and file.

        Returns:
            The local directory path string pointing to the chosen file.
        """
        map_categories: List[str] = ["easy", "medium", "hard", "challenger"]

        while True:
            category = input(
                "\nPlease enter a map category:\n"
                f"-{'-'.join([option + '\n' for option in map_categories])}\n"
            ).lower()

            if category not in map_categories:
                FlyIn.print_wrong_input(f"'{category}' is not a category")
                continue

            while True:
                maps: List[str] = sorted(listdir(f"./maps/{category}"))

                try:
                    selection = int(input(
                        "\nPlease enter the number of the map file:\n" +
                        f"{"\n".join(maps)}\n\n"
                    )) - 1
                except ValueError as e:
                    invalid_value: str = str(e).split("'")[1]
                    FlyIn.print_wrong_input(
                        f"'{invalid_value}' is not a number"
                    )
                    continue

                if selection not in range(len(maps)):
                    FlyIn.print_wrong_input(
                        f"Please enter a number from 1 to {len(maps)}"
                    )
                    continue
                else:
                    return f"./maps/{category}/{maps[selection]}"

    @staticmethod
    def print_wrong_input(message: str) -> None:
        """Prints a standardized validation error message to stdout.

        Args:
            message: The descriptive error text context to display.
        """
        print(f"\nWrong input: {message}.")

    @staticmethod
    def run() -> None:
        """Executes the core application loop.

        Loads the network map file, triggers path-finding optimizations and
        dispatches the rendering engine to visualize the finalized simulation.
        """
        map_file: str = FlyIn.get_file()
        print()

        try:
            graph = NetworkParser.load(map_file).to_graph()
            path_finding_algorithm = MinCostMaxFlowAlgorithm()
            path, drone_assignment, turns = (
                RoutePlanner(path_finding_algorithm, graph).schedule_drones()
            )
            simulation = SimulationEngine(path, drone_assignment, turns, graph)
            simulation_result = simulation.simulate_turns()
            print()
            Renderer.render_simulation(graph, simulation_result)
            simulation.print_secondary_metrics()
        except (NetworkFileError, FlowGraphError) as e: #ValueError too???
            print(f"Error: {e}")


if __name__ == "__main__":
    print("\n\n= = = = = = Welcome to Fly-In = = = = = =\n\n")
    fly_in = FlyIn.run()
