from typing import Dict


class NetworkFileError(Exception):
    pass


class NetworkFileNotFoundError(NetworkFileError):
    def __init__(self, file: str):
        super().__init__(f"Wrong input file: {file} doesn't exist.")


class ZoneConnectionNameError(NetworkFileError):
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Wrong format in line {line_n}: '{line}'. " +
            "Dashes and spaces are forbidden in zone names."
        )


class ZoneConnectionFormatError(NetworkFileError):
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Wrong format in line {line_n}: '{line}'. Zones must be " +
            "defined by a prefix ('start_hub', 'end_hub' or 'hub') as " +
            "prefix: <name> <x> <y> [metadata]. Connections must be " +
            "defined as connection: <name1>-<name2> [metadata]."
        )


class PrefixError(NetworkFileError):
    def __init__(self, prefix: str, line_n: int):
        super().__init__(
            f"Wrong prefix '{prefix}' in line {line_n}.\n" +
            "Prefix should be: 'nb_drones', 'start_hub', 'end_hub', " +
            "'hub' or 'connection'."
        )


class FirstLineError(NetworkFileError):
    def __init__(self, line: str, line_n: int):
        super().__init__(
            "First line of input file must define the number of drones with " +
            f"nb_drones: <positive_integer>. Now '{line}' in line {line_n}."
        )


class StarEndZoneError(NetworkFileError):
    def __init__(self, line: str, line_n: int, zone: str):
        super().__init__(
            f"Input file contains more than one '{zone}'. There must be " +
            f"exactly one. Please check '{line}' in line {line_n}."
        )


class MetadataFormatError(NetworkFileError):
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Wrong format in line {line_n}: '{line}'. Metadata blocks " +
            "must be defined as tag=value."
        )


class MetadataTagError(NetworkFileError):
    def __init__(self, line: str, line_n: int):
        super().__init__(
            f"Invalid metadata tag in line {line_n}: '{line}'. Allowed " +
            "tags for zones are 'zone', 'color' and 'max_drones'. Allowed " +
            "tag for connections is 'max_link_capacity'."
        )


class ModelValidationError(NetworkFileError):
    def __init__(self, line_message: Dict[int, str]):
        super().__init__(
            f"{
                '\n'.join([
                    f'{message} See line {line_n}.'
                    for line_n, message in line_message.items()
                ])
            }"
        )
