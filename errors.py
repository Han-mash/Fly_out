"""Custom exceptions for Fly-in project.
"""


class FlyInError(Exception):
    """
    Base class of all the project's own errors.
    """


class MapFileError(FlyInError):
    """
    The map file cannot be opened or read.

    Typical causes: missing file, directory instead of file,
    permission denied, invalid encoding.
    """


class ParseError(FlyInError):
    """
    The content of the map file is invalid.
    """

    def __init__(
        self,
        message: str,
        line_number: int | None = None,
        line: str | None = None
    ) -> None:
        super().__init__(message)
        self.message: str = message
        self.line_number: int | None = line_number
        self.line: str | None = line

    def __str__(self) -> str:
        if self.line_number is not None:
            details = f" (line {self.line_number})"
            if self.line:
                details += f": {self.line.strip()}"
            return f"Parse error{details} -> {self.message}"
        return f"Parse error: {self.message}"


class NoPathError(FlyInError):
    """
    No route exists between the start and the end zone.
    """


class SimulationError(FlyInError):
    """
    A produced schedule breaks a movement or occupancy rule.
    """


class DisplayError(FlyInError):
    """
    The graphical window cannot be opened.
    """
