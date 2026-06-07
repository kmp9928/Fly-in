from pydantic import BaseModel, Field, ValidationError, model_validator
from pydantic_core import PydanticCustomError
from typing import Optional, Self
from enum import Enum


class ZoneType(Enum):
    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"


class Color(Enum):
    NONE = "none"
    GREEN = "green"
    BLUE = "blue"
    RED = "red"
    YELLOW = "yellow"
    ORANGE = "orange"


class DronesN(BaseModel):
    number: int = Field(ge=1)
    line_n: int


class Node(BaseModel):
    name: str
    x: int = Field(ge=0)
    y: int
    zone: Optional[ZoneType] = Field(default=ZoneType.NORMAL)
    color: Optional[Color] = Field(default=Color.NONE)
    max_drones: Optional[int] = Field(default=1, ge=1)
    line_n: int

    @model_validator(mode='after')
    def check_name(self) -> Self:
        if "-" in self.name in self.name:
            raise ValidationError.from_exception_data(
                title="Network",
                line_errors=[{
                    "type": PydanticCustomError(
                        "invalid_zone_name_error",
                        "Dashes and spaces are forbidden in zone names. " +
                        f"Wrong value '{self.name}'"
                    ),
                    "loc": ("name", self.line_n)
                }]
            )
        return self


class Connection(BaseModel):
    from_hub: str
    to_hub: str
    max_link_capacity: Optional[int] = Field(default=1, ge=1)
    line_n: int

    @model_validator(mode='after')
    def check_name(self) -> Self:
        if "-" in self.from_hub or "-" in self.to_hub:
            raise ValidationError.from_exception_data(
                title="Network",
                line_errors=[{
                    "type": PydanticCustomError(
                        "invalid_conn_name_error",
                        "Dashes and spaces are forbidden in zone names. " +
                        f"Wrong value '{self.from_hub}-{self.to_hub}'"
                    ),
                    "loc": ("name", self.line_n)
                }]
            )
        return self
