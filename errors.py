from typing import Dict


class NetworkFileError(Exception):
    """Base exception for configuration file parsing/formatting violations."""
    pass


class NetworkFileNotFoundError(NetworkFileError):
    """Raised when the requested network configuration file cannot be located.

    Args:
        file: The system path or filename string that was not found.
    """
    def __init__(self, file: str):
        super().__init__(f"Wrong input file: {file} doesn't exist.")


class ZoneConnectionNameError(NetworkFileError):
    """Raised when a hub or connection string contains forbidden characters.

    Args:
        line: The raw string content of the offending configuration line.
        line_n: The 1-based index position line number in the source file.
    """
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Wrong format in line {line_n}: '{line}'. " +
            "Dashes and spaces are forbidden in zone names."
        )


class ZoneConnectionFormatError(NetworkFileError):
    """Raised when a hub or connection line breaks structural syntax patterns.

    Args:
        line: The raw string content of the offending configuration line.
        line_n: The 1-based index position line number in the source file.
    """
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Wrong format in line {line_n}: '{line}'. Zones must be " +
            "defined by a prefix ('start_hub', 'end_hub' or 'hub') as " +
            "prefix: <name> <x> <y> [metadata]. Connections must be " +
            "defined as connection: <name1>-<name2> [metadata]."
        )


class PrefixError(NetworkFileError):
    """Raised when an unidentifiable row keyword prefix token is encountered.

    Args:
        prefix: The invalid text token sequence preceding the ': ' separator.
        line_n: The 1-based index position line number in the source file.
    """
    def __init__(self, prefix: str, line_n: int):
        super().__init__(
            f"Wrong prefix '{prefix}' in line {line_n}.\n" +
            "Prefix should be: 'nb_drones', 'start_hub', 'end_hub', " +
            "'hub' or 'connection'."
        )


class FirstLineError(NetworkFileError):
    """Raised when other data parameters precede the drone number line.

    Args:
        line: The raw string content of the offending configuration line.
        line_n: The 1-based index position line number in the source file.
    """
    def __init__(self, line: str, line_n: int):
        super().__init__(
            "First line of input file must define the number of drones with " +
            f"nb_drones: <positive_integer>. Now '{line}' in line {line_n}."
        )


class StarEndZoneError(NetworkFileError):
    """Raised when duplicate definitions are encountered for boundary hubs.

    Args:
        line: The raw string content of the offending configuration line.
        line_n: The 1-based index position line number in the source file.
        zone: The keyword that was duplicated ('start_hub'/'end_hub').
    """
    def __init__(self, line: str, line_n: int, zone: str):
        super().__init__(
            f"Input file contains more than one '{zone}'. There must be " +
            f"exactly one. Please check '{line}' in line {line_n}."
        )


class MetadataFormatError(NetworkFileError):
    """Raised when bracketed metadata values violate the syntax standard.

    Args:
        line: The raw string that failed syntax separation rules.
        line_n: The 1-based index position line number in the source file.
    """
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Wrong format in line {line_n}: '{line}'. Metadata blocks " +
            "must be defined as tag=value."
        )


class MetadataTagError(NetworkFileError):
    """Raised when a non-existent/unmapped parameter property key tag is used.

    Args:
        line: The raw token assignment block string that contained the bad tag.
        line_n: The 1-based index position line number in the source file.
    """
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Invalid metadata tag in line {line_n}: '{line}'. Allowed " +
            "tags for zones are 'zone', 'color' and 'max_drones'. Allowed " +
            "tag for connections is 'max_link_capacity'."
        )


class ModelValidationError(NetworkFileError):
    """Wraps Pydantic schema exceptions sorted and mapped by source file line.

    Args:
        line_message: A dictionary mapping source file lines to their
            validation error text.
    """
    def __init__(self, line_message: Dict[int, str]):
        super().__init__(
            f"{
                '\n'.join([
                    f'{message}' if line_n == 0
                    else f'{message} See line {line_n}.'
                    for line_n, message in line_message.items()
                ])
            }"
        )


class MissingMandatoryFieldError(NetworkFileError):
    """Raised when a mandatory prefix is missing from the input file.

    Args:
        missing: The missing prefix.
    """
    def __init__(self, missing: str):
        super().__init__(
            f"Missing mandatory prefix {missing} in input file."
        )


class GraphError(Exception):
    """Base exception for errors inside the network graph."""
    pass


class FlowGraphError(Exception):
    """Base exception for errors inside the flow graph."""
    pass
