from pydantic import BaseModel, ValidationError, model_validator
from pydantic_core import PydanticCustomError
from typing import Tuple, List, Dict, Any, Self
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
    MissingMandatoryFieldError
)
from models import ZoneType, Color, DronesN, Node, Connection
from graph import Graph


class Network(BaseModel):
    """Represents a digital map of the entire drone setup.

    Validates spatial layout schemas, entity naming criteria, structural
    uniqueness limits and cross-references connectivity chains across
    interconnecting drone infrastructure components.

    Attributes:
        nb_drones (DronesN): Total number of drones in the network.
        start_hub (Node): The designated source/entry point node.
        end_hub (Node): The designated terminal destination/drop off node.
        hubs (List[Node]): Collection of intermediate routing staging nodes.
        connections (List[Connection]): Edge or connection between two nodes.
    """
    nb_drones: DronesN
    start_hub: Node
    end_hub: Node
    hubs: List[Node]
    connections: List[Connection]

    @model_validator(mode='after')
    def change_start_end_max_drones(self) -> Self:
        """Synchronizes drone threshold capacities across entry and exit hubs.

        Returns:
            Self: The updated structural state model reference instance.
        """
        self.start_hub.max_drones = self.nb_drones.number
        self.end_hub.max_drones = self.nb_drones.number
        return self

    @model_validator(mode='after')
    def compare_start_vs_end(self) -> Self:
        """Verifies terminal hub spatial separation boundaries.

        Returns:
            Self: The verified baseline structural data instance.

        Raises:
            ValidationError: If start and end hubs share identical coordinates.
        """
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
                    "loc": ("end_hub", "x and y"),
                    "input": {
                        "start_x": self.start_hub.x, "end_x": self.end_hub.x
                    }
                }]
            )
        return self

    @model_validator(mode='after')
    def check_zones_coordinates(self) -> Self:
        """Validates that intermediate hubs are within grid boundaries.

        Returns:
            Self: The validated model instance.

        Raises:
            ValidationError: If an intermediate hub's x coordinate falls
                outside the horizontal interval between the start and end hubs.
        """
        for n, zone in enumerate(self.hubs, start=0):
            if zone.x < self.start_hub.x or zone.x > self.end_hub.x:
                raise ValidationError.from_exception_data(
                    title="Network",
                    line_errors=[{
                        "type": PydanticCustomError(
                            "invalid_coord_error",
                            f"'{zone.name}' has invalid integer coordinates"
                        ),
                        "loc": ("hubs", n),
                        "input": {"zone": zone.name, "x": zone.x}
                    }]
                )
        return self

    @model_validator(mode='after')
    def check_zone_uniqueness(self) -> Self:
        """Ensures all hubs/nodes across the network are completely unique.

        Returns:
            Self: The validated structural context model configuration.

        Raises:
            ValidationError: If an identifier collision is found across any
                combination of start, intermediate, or end hubs.
        """
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
                        "loc": ("hubs", n),
                        "input": zone
                    }]
                )
        return self

    @model_validator(mode='after')
    def check_connections_existence(self) -> Self:
        """Validates that the network has not just hubs.

        Returns:
            Self: The validated baseline model configuration instance.

        Raises:
            ValidationError: If there are no connections.
        """
        if len(self.connections) < 1:
            raise ValidationError.from_exception_data(
                title="Network",
                line_errors=[{
                    "type": PydanticCustomError(
                        "no_connections_error",
                        "Missing connections in network file"
                    ),
                    "loc": ("connections",),
                    "input": self.connections
                }]
            )
        return self

    @model_validator(mode='after')
    def check_connections(self) -> Self:
        """Validates that all connections point to real, existing hubs.

        Returns:
            Self: The validated baseline model configuration instance.

        Raises:
            ValidationError: If a connection string links to an unrecognized
                zone name.
        """
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
                        "loc": ("connections", n),
                        "input": f"{name1}-{name2}"
                    }]
                )
        return self

    @model_validator(mode='after')
    def check_connection_uniqueness(self) -> Self:
        """Prevents duplicate connections, ignoring path direction.

        Returns:
            Self: The checked system model representation state.

        Raises:
            ValidationError: If an undirected connection link structure
                duplicate is encountered in the dataset setup list.
        """
        unique_connections: List[Tuple[str, str]] = []
        for n, connection in enumerate(self.connections):
            name1, name2 = sorted((connection.from_hub, connection.to_hub))
            normalized_pair: Tuple[str, str] = (name1, name2)
            if normalized_pair in unique_connections:
                raise ValidationError.from_exception_data(
                    title="Network",
                    line_errors=[{
                        "type": PydanticCustomError(
                            "duplicate_conn_error",
                            f"Connection {normalized_pair} is duplicated"
                        ),
                        "loc": ("connections", n),
                        "input": normalized_pair
                    }]
                )
            unique_connections.append(normalized_pair)
        return self

    def to_graph(self) -> Graph:
        """Transforms data parameters into a graph layout object.

        Returns:
            Graph: An instance of the Graph visualization network matrix.
        """
        return Graph(
            start=self.start_hub,
            end=self.end_hub,
            nodes=self.hubs,
            connections=self.connections,
            drones=self.nb_drones
        )


class NetworkParser:
    """Parsing class that turns configuration files into Network models.

    Processes raw plain text lines, sanitizes structural components,
    handles domain-specific syntax exceptions and catches metadata tags.
    """
    @staticmethod
    def load(file_name: str) -> Network:
        """Parses a target text configuration file into a Network object.

        Args:
            file_name: The file of the network of drones.

        Returns:
            Network: A parsed, validated Pydantic network schema instance.

        Raises:
            NetworkFileNotFoundError: If the designated file is missing.
            ModelValidationError: If any formatting validation criteria fails.
            FirstLineError: If the first line doesn't define the drone number.
            StarEndZoneError: If more than one start or end are found.
        """
        hubs: List[Dict[str, str | int | ZoneType | Color]] = []
        connections: List[Dict[str, str | int]] = []
        data: Dict[str, Any] = {"hubs": hubs, "connections": connections}
        try:
            with open(file_name, "r") as file:
                for line_n, line_str in enumerate(file, start=1):
                    line = line_str.strip()
                    if line.startswith("#") or not line:
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
                        data["hubs"].append(
                            NetworkParser.parse_zone((value, line_n))
                        )
                    elif pfx == "connection":
                        data["connections"].append(
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
        except KeyError as e:
            raise MissingMandatoryFieldError(str(e))

    @staticmethod
    def get_prefix_value(line: str, line_n: int) -> Tuple[str, str]:
        """Splits structural row strings into key-value pairs.

        Args:
            line: The raw unparsed text row string.
            line_n: The current context execution line number.

        Returns:
            Tuple[str, str]: A split pair matching (prefix_key, string_value).

        Raises:
            ZoneConnectionFormatError: If separator ': ' is missing/duplicated.
            PrefixError: If an unknown prefix is found.
        """
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
        is_hub: bool, metadata: str, line_n: int
    ) -> Dict[str, str]:
        """Parses optional attribute strings into type-mapped dictionaries.

        Args:
            is_hub: True if parsing a hub/node, False for paths/connections.
            metadata: Raw internal attributes.
            line_n: File tracker position index string context reference.

        Returns:
            Dict[str, str]: Mapped metadata tags.

        Raises:
            MetadataFormatError: If an assignment block is incorrect.
            MetadataTagError: If an invalid tag is encountered.
        """
        clean_metadata: Dict[str, str] = {}
        if metadata:
            split_metadata = metadata.strip("]").split()
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
    def parse_zone(zone_info: Tuple[str, int]) -> Dict[str, str | int]:
        """Extracts identifiers and parameters for a network hub.

        Args:
            zone_info: A tuple container combining (node_string, line_number).

        Returns:
            Dict[str, str]: Extracted zone property arguments map.

        Raises:
            ZoneConnectionNameError: If components violate syntax patterns.
            ZoneConnectionFormatError: If definitions are missing.
        """
        info, line_n = zone_info
        all_info = info.split("[")
        name_x_y = all_info[0].split()

        if len(name_x_y) > 3:
            raise ZoneConnectionNameError(info, line_n)
        elif len(name_x_y) < 3:
            raise ZoneConnectionFormatError(info, line_n)

        name, x, y = name_x_y
        zone_args: Dict[str, str | int] = {
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
        """Processes link data fields connecting different graph nodes.

        Args:
            connection: Tuple pair packing (connection_string, line_number).

        Returns:
            Dict[str, str]: Connection mapping schema values.

        Raises:
            ZoneConnectionNameError: If components violate syntax patterns.
            ZoneConnectionFormatError: If definitions are missing.
        """
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
        """Maps Pydantic validation errors back to their file line numbers.

        Args:
            e: The validation exception object triggered by the Network model.
            data: The internal temporary layout from the source code data.

        Returns:
            Dict[int, str]: A line-number-sorted map of user-friendly
                validation error messages.
        """
        validation_errors: Dict[int, str] = {}
        for error in e.errors():
            location = error["loc"]
            if location:
                key = location[0]
                if (
                    isinstance(key, str) and
                    key in ["nb_drones", "start_hub", "end_hub"]
                ):
                    line_n = data[key].get("line_n")
                    validation_errors[line_n] = (
                        f"{error['msg']} for '{location[1]}'."
                    )
                elif key in ["hubs", "connections"]:
                    if len(location) > 1:
                        index = location[1]
                        if isinstance(key, str) and isinstance(index, int):
                            line_n = data[key][index].get("line_n")
                            if len(location) <= 2:
                                validation_errors[line_n] = (
                                    f"{error['msg']}."
                                )
                            else:
                                validation_errors[line_n] = (
                                    f"{error['msg']} for '{location[2]}'."
                                )
                    else:
                        validation_errors[0] = (f"{error['msg']}.")

        return ({k: v for k, v in sorted(
            validation_errors.items(), key=lambda item: item[0]
            )
        })
