from pydantic import BaseModel, Field, ValidationError, model_validator
from pydantic_core import PydanticCustomError
from typing import Self
from enum import Enum


class ZoneType(Enum):
    """Defines the operational status and behavior profile of a map hub.

    Attributes:
        NORMAL: Standard waypoint node with standard routing behaviors.
        BLOCKED: Completely inaccessible zone closed to all drone traffic.
        RESTRICTED: Limited-access zone requiring special routing clearance.
        PRIORITY: High-importance node favored for path planning optimizations.
    """
    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"


class Color(Enum):
    """Specifies the display color tokens for node visualization rendering.

    Contains standard color spectrum values alongside specific fallback and
    dynamic structural presets (e.g., NONE, RAINBOW).
    """
    NONE = "none"
    BLACK = "black"
    BLUE = "blue"
    BROWN = "brown"
    CRIMSON = "crimson"
    CYAN = "cyan"
    DARKRED = "darkred"
    GOLD = "gold"
    GREEN = "green"
    LIME = "lime"
    MAGENTA = "magenta"
    MAROON = "maroon"
    ORANGE = "orange"
    PURPLE = "purple"
    RAINBOW = "rainbow"
    RED = "red"
    TEAL = "teal"
    VIOLET = "violet"
    WHITE = "white"
    YELLOW = "yellow"


class DronesN(BaseModel):
    """Validates the count configuration setting for active simulation drones.

    Attributes:
        number (int): Total number of drones allowed in the network schema.
        line_n (int): Source file tracker position index line counter.
    """
    number: int = Field(ge=1)
    line_n: int


class Node(BaseModel):
    """Represents a coordinate hub/node within the simulation network map grid.

    Handles localized drone constraints, visual render styles, operational
        classifications and checks text token character compliance.

    Attributes:
        name (str): The structural identifier string for the hub point/node.
        x (int): Horizontal spatial position on the grid matrix layout.
        y (int): Vertical spatial position on the grid matrix layout.
        zone (ZoneType): Flight behavior zone profile.
        color (Color): Visual map marker rendering token.
        max_drones (int): Capacity limit parameter for drones.
        line_n (int): Source file tracker position index line counter.
    """
    name: str
    x: int = Field(ge=0)
    y: int
    zone: ZoneType = Field(default=ZoneType.NORMAL) #removed optional and set default to normal
    color: Color = Field(default=Color.NONE) #removed optional and set default to none...check renderer!!!
    max_drones: int = Field(default=1, ge=1) #removed optional and set default to 1
    line_n: int

    @model_validator(mode='after')
    def check_name(self) -> Self:
        """Enforces naming criteria on the hub identifier token.

        Returns:
            Self: The validated baseline model structure data instance.

        Raises:
            ValidationError: If the node name contains dashes/spaces.
        """
        if "-" in self.name in self.name:
            raise ValidationError.from_exception_data(
                title="Network",
                line_errors=[{
                    "type": PydanticCustomError(
                        "invalid_zone_name_error",
                        "Dashes and spaces are forbidden in zone names. " +
                        f"Wrong value '{self.name}'"
                    ),
                    "loc": ("name", self.line_n),
                    "input": self.name #new...see if issues
                }]
            )
        return self


class Connection(BaseModel):
    """Defines an edge/path corridor linking two individual network hubs.

    Tracks how many drones can use a path at the same time and makes sure
        the connected hub names don't use illegal characters.

    Attributes:
        from_hub (str): Identifier of the departure/origin node hub.
        to_hub (str): Identifier of the destination/target node hub.
        max_link_capacity (int): Max count of drones allowed on path.
        line_n (int): Source file tracker position index line counter.
    """
    from_hub: str
    to_hub: str
    max_link_capacity: int = Field(default=1, ge=1) #removed optional and set default to 1
    line_n: int

    @model_validator(mode='after')
    def check_name(self) -> Self:
        """Validates that edge labels do not contain illegal characters.

        Returns:
            Self: The validated schema path model data structure reference.

        Raises:
            ValidationError: If either linked hub contains a forbidden dash.
        """
        if "-" in self.from_hub or "-" in self.to_hub:
            raise ValidationError.from_exception_data(
                title="Network",
                line_errors=[{
                    "type": PydanticCustomError(
                        "invalid_conn_name_error",
                        "Dashes and spaces are forbidden in zone names. " +
                        f"Wrong value '{self.from_hub}-{self.to_hub}'"
                    ),
                    "loc": ("name", self.line_n),
                    "input": f"{self.from_hub}-{self.to_hub}" #new...see if issues
                }]
            )
        return self
