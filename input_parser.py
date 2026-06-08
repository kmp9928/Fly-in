from pydantic import BaseModel, ValidationError, model_validator
from pydantic_core import PydanticCustomError
from typing import Tuple, List, Dict, Any, Self, Set
from errors import (
    NetworkFileNotFoundError,
    ZoneConnectionNameError,
    ZoneConnectionFormatError,
    PrefixError,
    FirstLineError,
    StarEndZoneError,
    MetadataFormatError,
    MetadataTagError,
    ModelValidationError,
)
from models import ZoneType, Color, DronesN, Node, Connection
from graph import Graph


class Network(BaseModel):
    nb_drones: DronesN
    start_hub: Node
    end_hub: Node
    hubs: List[Node]
    connections: List[Connection]

    @model_validator(mode='after')
    def change_start_end_max_drones(self) -> Self:
        self.start_hub.max_drones = self.nb_drones.number
        self.end_hub.max_drones = self.nb_drones.number
        return self

    @model_validator(mode='after')
    def compare_start_vs_end(self) -> Self:
        if (
            self.start_hub.x == self.end_hub.x and
            self.start_hub.y == self.end_hub.y
        ):
            raise ValidationError.from_exception_data(
                title="Network",
                line_errors=[{
                    "type": PydanticCustomError(
                        "start_end_error",
                        "Start and end zone have the same coordinates"
                    ),
                    "loc": ("end_hub", "x and y")
                }]
            )
        return self

    @model_validator(mode='after')
    def check_zones_coordinates(self) -> Self:
        for n, zone in enumerate(self.hubs, start=0):
            if zone.x < self.start_hub.x or zone.x > self.end_hub.x:
                raise ValidationError.from_exception_data(
                    title="Network",
                    line_errors=[{
                        "type": PydanticCustomError(
                            "invalid_coord_error",
                            f"'{zone.name}' has invalid integer coordinates"
                        ),
                        "loc": ("hubs", n)
                    }]
                )
        return self

    @model_validator(mode='after')
    def check_zone_uniqueness(self) -> Self:
        zones: List[str] = [hub.name for hub in self.hubs]
        zones = [self.start_hub.name] + zones + [self.end_hub.name]
        for n, zone in enumerate(zones, start=0):
            if zones.count(zone) > 1:
                raise ValidationError.from_exception_data(
                    title="Network",
                    line_errors=[{
                        "type": PydanticCustomError(
                            "duplicate_zone_error",
                            f"Zone '{zone}' is not unique"
                        ),
                        "loc": ("hubs", n)
                    }]
                )
        return self

    @model_validator(mode='after')
    def check_connections(self) -> Self:
        zones: List[str] = [hub.name for hub in self.hubs]
        zones.extend([self.start_hub.name, self.end_hub.name])
        for n, connection in enumerate(self.connections, start=0):
            name1 = connection.from_hub
            name2 = connection.to_hub
            if name1 not in zones or name2 not in zones:
                raise ValidationError.from_exception_data(
                    title="Network",
                    line_errors=[{
                        "type": PydanticCustomError(
                            "undefined_zone_error",
                            f"Connection {name1}-{name2} has undefined zone"
                        ),
                        "loc": ("connections", n)
                    }]
                )
        return self

    @model_validator(mode='after')
    def check_connection_uniqueness(self) -> Self: #changed to somethign cleaner - test!!!!!!
        unique_connections: List[Tuple[str, str]] = []
        for n, connection in enumerate(self.connections):
            normalized_pair = sorted((connection.from_hub, connection.to_hub))
            if normalized_pair in unique_connections:
                raise ValidationError.from_exception_data(
                    title="Network",
                    line_errors=[{
                        "type": PydanticCustomError(
                            "duplicate_conn_error",
                            f"Connection {normalized_pair} is duplicated"
                        ),
                        "loc": ("connections", n)
                    }]
                )
            unique_connections.append(normalized_pair)
        return self

    def to_graph(self) -> Graph:
        return Graph(
            start=self.start_hub,
            end=self.end_hub,
            nodes=self.hubs,
            connections=self.connections,
            drones=self.nb_drones
        )


class NetworkParser:
    @staticmethod
    def load(file_name: str) -> Network:
        hubs: List[Dict[str, str | int | ZoneType | Color]] = []
        connections: List[Dict[str, str | int]] = []
        data: Dict[str, Any] = {"hubs": hubs, "connections": connections}
        try:
            with open(file_name, "r") as file:
                for line_n, line in enumerate(file, start=1):
                    if line.startswith("#") or not line.strip():
                        continue
                    pfx, value = NetworkParser.get_prefix_value(line, line_n)
                    if (pfx != "nb_drones" and len(data) == 2 and
                            all(values == [] for values in data.values())):
                        raise FirstLineError(line, line_n)
                    if pfx in ["start_hub", "end_hub"] and pfx in data.keys():
                        raise StarEndZoneError(line, line_n, pfx)
                    if pfx == "nb_drones":
                        data[pfx] = {"number": value, "line_n": line_n}
                    elif pfx in ["start_hub", "end_hub"]:
                        data[pfx] = NetworkParser.parse_zone((value, line_n))
                    elif pfx == "hub":
                        data.get("hubs").append(
                            NetworkParser.parse_zone((value, line_n))
                        )
                    elif pfx == "connection":
                        data.get("connections").append(
                            NetworkParser.parse_connection((value, line_n))
                        )
            return Network(
                nb_drones=data["nb_drones"],
                start_hub=data["start_hub"],
                end_hub=data["end_hub"],
                hubs=data["hubs"],
                connections=data["connections"]
            )
        except FileNotFoundError as e:
            raise NetworkFileNotFoundError(str(e).split(": ")[1])
        except ValidationError as e:
            raise ModelValidationError(
                NetworkParser.get_validation_errors(e, data)
            )

    @staticmethod
    def get_prefix_value(line: str, line_n: int) -> Tuple[str, str]:
        pfx_value = line.split(": ")
        if len(pfx_value) != 2:
            raise ZoneConnectionFormatError(line, line_n)
        if pfx_value[0] not in [
            "nb_drones", "start_hub", "end_hub", "hub", "connection"
        ]:
            raise PrefixError(pfx_value[0], line_n)
        return (pfx_value[0], pfx_value[1].strip())

    @staticmethod
    def parse_metadata(
        is_hub: bool, metadata: str, line_n: str
    ) -> Dict[str, int | ZoneType | Color]:
        clean_metadata: Dict[str, ZoneType | Color | int] = {}
        if metadata:
            split_metadata = metadata.strip("[]").split()
            for element in split_metadata:
                try:
                    tag, value = element.split("=")
                except ValueError:
                    raise MetadataFormatError(element, line_n)
                if (
                    (is_hub and tag not in ["zone", "color", "max_drones"]) or
                    not is_hub and tag != "max_link_capacity"
                ):
                    raise MetadataTagError(element, line_n)
                clean_metadata[tag] = value
        return clean_metadata

    @staticmethod
    def parse_zone(
        zone_info: Tuple[str, int]
    ) -> Dict[str, str | int | ZoneType | Color]:
        info, line_n = zone_info
        all_info = info.split("[")
        name_x_y = all_info[0].split()

        if len(name_x_y) > 3:
            raise ZoneConnectionNameError(info, line_n)
        elif len(name_x_y) < 3:
            raise ZoneConnectionFormatError(info, line_n)

        name, x, y = name_x_y
        zone_args: Dict[str, str | int | ZoneType | Color] = {
            "name": name,
            "x": x,
            "y": y,
            "line_n": line_n
        }
        if len(all_info) > 1:
            zone_args.update(
                NetworkParser.parse_metadata(True, all_info[1], line_n)
            )
        return zone_args

    @staticmethod
    def parse_connection(connection: Tuple[str, int]) -> Dict[str, str | int]:
        info, line_n = connection
        all_info = info.split("[")
        from_to = all_info[0].strip().split("-")

        if len(from_to) > 2:
            raise ZoneConnectionNameError(info, line_n)
        if len(from_to) < 2:
            raise ZoneConnectionFormatError(info, line_n)

        from_hub, to_hub = from_to
        connection_args: Dict[str, str | int] = {
            "from_hub": from_hub, "to_hub": to_hub, "line_n": line_n
        }
        if len(all_info) > 1:
            connection_args.update(
                NetworkParser.parse_metadata(False, all_info[1], line_n)
            )
        return connection_args

    @staticmethod
    def get_validation_errors(
        e: ValidationError, data: Dict[str, Any]
    ) -> Dict[int, str]:
        validation_errors: Dict[int, str] = {}
        for error in e.errors():
            location = error["loc"]
            if location:
                if location[0] in ["nb_drones", "start_hub", "end_hub"]:
                    line_n = data[location[0]].get("line_n")
                    validation_errors[line_n] = (
                        f"{error['msg']} for '{location[1]}'."
                    )
                elif location[0] in ["hubs", "connections"]:
                    line_n = data[location[0]][location[1]].get("line_n")
                    if len(location) <= 2:
                        validation_errors[line_n] = (f"{error['msg']}.")
                    else:
                        validation_errors[line_n] = (
                            f"{error['msg']} for '{location[2]}'."
                        )
        return ({k: v for k, v in sorted(
            validation_errors.items(), key=lambda item: item[0]
            )
        })


# if __name__ == "__main__":
#     try:
#         parser = NetworkParser.load("03_priority_puzzle.txt")
#         print(f"nb_drones is: {parser.nb_drones}\n")
#         print(f"start_hub is: {parser.start_hub}\n")
#         print(f"end_hub is: {parser.end_hub}\n")
#         for n, hub in enumerate(parser.hubs, start=1):
#             print(f"hub {n} is: {hub}\n")
#         for n, connection in enumerate(parser.connections, start=1):
#             print(f"connection {n} is: {connection}\n")
#     except NetworkFileError as e:
#         print(e)

#check in call ValueError for wrong def of any elemnt above (like no name or missing x or y or field not defined in model)...
# check ValueError
#what about passing str in max_link_capacity, zone clor or max_drones?
#what are valid integer coord? nothing can be firther than end?
#check types of zones throw error when not valid..or any enum
#maybe data{} should also save line num
# max_drones should be default to nb_drones for start and end hub???
#start_hub and end_hub should not have movement cost??? so even if it's set to normal check execution