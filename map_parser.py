"""Parser for Fly-in map files.
"""

from enum import Enum
from typing import Iterable

from errors import MapFileError, ParseError
from graph import Graph
from models import Zone, ZoneType, Connection


class LineKind(Enum):
    """The five kinds of significant lines of a map file."""

    NB_DRONES = "nb_drones"
    START_HUB = "start_hub"
    END_HUB = "end_hub"
    HUB = "hub"
    CONNECTION = "connection"

    @classmethod
    def from_string(cls, value: str) -> "LineKind":
        """Convert the text before the first colon into a LineKind.

        Raises:
            ValueError: If the prefix is unknown.
        """
        normalized = value.strip()
        for kind in cls:
            if kind.value == normalized:
                return kind
        excepted = ", ".join(kind.value for kind in cls)
        raise ValueError(
            f"Unknown line kind {value!r} (expected one of {excepted})"
            )


class MapParser:
    """Turn a map file into a validated Graph.

    The parser RAISES ParseError / MapFileError. It never prints and
    never exits: the entry point decides what to do with the error.
    """

    ZONE_KEYS: frozenset[str] = frozenset({"zone", "color", "max_drones"})
    CONNECTION_KEYS: frozenset[str] = frozenset({"max_link_capacity"})

    def __init__(self) -> None:
        """Create a parser.
        The graph being built, the current line
        number and raw text (for error messages), and a flag telling
        whether nb_drones was already read."""
        self._graph: Graph = Graph()
        self._line_number: int = 0
        self._raw_line: str = ""
        self._nb_drones_seen: bool = False

    def parse_file(self, path: str) -> Graph:
        """Read a map file and return the graph.

        Raises:
            MapFileError: If the file cannot be opened or decoded.
            ParseError: If the content is invalid.
        """
        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = f.readlines()
        except (OSError, UnicodeDecodeError) as e:
            raise MapFileError(f"cannot read <{path}>.") from e
        return self.parse_lines(lines)

    def parse_lines(self, lines: Iterable[str]) -> Graph:
        """Parse map lines already in memory.

        Args:
            lines: Raw lines of a map file.

        Returns:
            The validated graph

        Raises:
            ParseError: If the content is invalid.
        """
        self._reset()
        for number, raw in enumerate(lines, start=1):
            self._line_number = number
            self._raw_line = raw
            line = self._clean_line(raw)
            if not line:
                continue
            kind, payload = self._split_kind(line)
            if not self._nb_drones_seen and kind != LineKind.NB_DRONES:
                raise self._error("nb_drones must come first.")
            if self._nb_drones_seen and kind == LineKind.NB_DRONES:
                raise self._error("duplicate nb_drones.")
            if kind == LineKind.NB_DRONES:
                self._handle_nb_drones(payload)
            elif kind == LineKind.CONNECTION:
                self._handle_connection(payload)
            else:
                self._handle_zone(kind, payload)
        self._check_end_of_file()
        return self._graph

    def _reset(self) -> None:
        """Forget everything about a previous parsing."""
        self._graph = Graph()
        self._line_number = 0
        self._raw_line = ""
        self._nb_drones_seen = False

    def _clean_line(self, raw: str) -> str:
        """Remove the comment and surrounding spaces.

        Returns:
            The significant text, or "" for blank / comment lines.
        """
        first_part = raw.partition("#")[0]
        return first_part.strip()

    def _split_kind(self, line: str) -> tuple[LineKind, str]:
        """Split a cleaned line into its kind and its payload.

        Raises:
            ParseError: If there is no colon or the prefix is unknown.
        """
        if ":" not in line:
            raise self._error(
                "missing ':' (expected '<kind>: ...')."
                )
        first_part, _, payload = line.partition(":")
        try:
            kind = LineKind.from_string(first_part.strip())
        except ValueError as e:
            raise self._error(str(e)) from e
        return (kind, payload.strip())

    def _split_metadata(
        self, payload: str, allowed_keys: frozenset[str]
    ) -> tuple[str, dict[str, str]]:
        """Separate the body of a line from its optional ``[...]``.

        Args:
            payload: Text after the prefix.
            allowed_keys: Metadata keys accepted for this line kind.

        Returns:
            (body, metadata) where metadata maps key -> raw value.

        Raises:
            ParseError: Unclosed bracket, text after ``]``, tag
                without ``=``, empty key / value, unknown key,
                duplicated key.
        """
        body = payload.strip()
        metadata: dict[str, str] = {}
        opening = body.count("[")
        closing = body.count("]")

        if opening == 0 and closing == 0:
            return body, metadata
        if opening == 0:
            raise self._error("Invalid Metadata - unexpected ']' without '['.")
        if closing == 0:
            raise self._error("Invalid Metadata - unclosed bracket.")
        if opening != 1 or closing != 1:
            raise self._error(
                "Invalid Metadata - metadata needs exactly one '[' and one ']'"
                )
        before = body.index("[")
        after = body.rindex("]")
        if after < before:
            raise self._error("Invalid Metadata - ']' found before '['.")
        if body[after + 1:].strip():
            raise self._error("Invalid Metadata - unexpected text after ']'.")

        content = body[before + 1:after].strip()
        body = body[:before].strip()

        if not content:
            return body, metadata
        for item in content.split():
            if "=" not in item:
                raise self._error(f"Invalid metadata ({item!r}).")
            key, _, value = item.partition("=")
            key = key.strip()
            value = value.strip()

            if not key or not value:
                raise self._error("Empty metadata (key or value missing).")
            if key not in allowed_keys:
                expected = ", ".join(sorted(allowed_keys))
                raise self._error(
                    f"Unknown metadata key: {key!r} "
                    f"(expected one of: {expected}).")
            if key in metadata:
                raise self._error(f"Duplicated metadata key: {key!r}.")
            metadata[key] = value
        return body, metadata

    def _handle_nb_drones(self, payload: str) -> None:
        """Process a ``nb_drones:`` line (must be the first one)."""
        count = self._to_positive_int(payload, "nb_drones")
        self._graph.set_nb_drones(count)
        self._nb_drones_seen = True

    def _handle_zone(self, kind: LineKind, payload: str) -> None:
        """Process a start_hub / end_hub / hub line.

        Checks: 3 fields (name x y), valid name, integer coordinates,
        valid zone type, positive max_drones, one-word colour.
        """
        body, metadata = self._split_metadata(payload, self.ZONE_KEYS)
        if len(body.split()) != 3:
            raise self._error("Invalid data - expected <name> <x> <y>")
        name, x_text, y_text = body.split()
        if not Zone.is_valid_name(name):
            raise self._error(f"Invalid Zone - {name!r}")
        x = self._to_int(x_text, "x")
        y = self._to_int(y_text, "y")
        try:
            zone_type = ZoneType.from_string(metadata.get("zone", "normal"))
        except ValueError as e:
            raise self._error(str(e)) from e

        d = "max_drones"
        if d in metadata:
            max_drones = self._to_positive_int(metadata[d], d)
        else:
            max_drones = 1

        try:
            zone = Zone(name, x, y, zone_type, metadata.get("color"),
                        max_drones,
                        is_start=(kind == LineKind.START_HUB),
                        is_end=kind == LineKind.END_HUB)
            self._graph.add_zone(zone)
        except ValueError as e:
            raise self._error(str(e)) from e

    def _handle_connection(self, payload: str) -> None:
        """Process a ``connection:`` line.

        Checks: exactly one dash, both zones already defined,
        no duplicate (a-b / b-a), positive max_link_capacity.

        Args:
            payload: Text after ``connection:``.

        """
        body, metadata = self._split_metadata(
            payload, self.CONNECTION_KEYS
        )
        if len(body.split()) != 1 or body.count("-") != 1:
            raise self._error(
                f"expected '<zone1>-<zone2>', got {body!r}."
            )
        name_a, name_b = body.split("-")
        if not name_a or not name_b:
            raise self._error(f"empty zone name in {body!r}.")
        for name in (name_a, name_b):
            if not self._graph.has_zone(name):
                raise self._error(
                    f"unknown zone {name!r}: zone must be defined "
                    "before the connection"
                )
        if self._graph.has_connection(name_a, name_b):
            raise self._error(
                f"duplicate connection {name_a, name_b}"
            )
        capacity = 1
        if "max_link_capacity" in metadata:
            capacity = self._to_positive_int(
                metadata["max_link_capacity"], "max_link_capacity"
            )
        try:
            connection = Connection(
                self._graph.get_zone(name_a),
                self._graph.get_zone(name_b),
                capacity
            )
            self._graph.add_connection(connection)
        except ValueError as e:
            raise self._error(str(e)) from e

    def _to_int(self, text: str, what: str) -> int:
        """Convert text to int.

        Raises:
            ParseError: With a message naming ``what``.
        """
        digits = text[1:] if text.startswith("-") else text
        if not (digits.isascii() and digits.isdigit()):
            raise self._error(
                f"{what} must be an integer, got {text!r}."
                )
        return int(text)

    def _to_positive_int(self, text: str, what: str) -> int:
        """Convert text to an int >= 1.

        Raises:
            ParseError: With a message naming ``what``.
        """
        value = self._to_int(text, what)
        if value < 1:
            raise self._error(
                f"{what} must be a positive integer, got {value}."
            )
        return value

    def _check_end_of_file(self) -> None:
        """Checks that need the whole file.

        Missing nb_drones, missing start_hub, missing end_hub.
        These errors have no meaningful line number.
        """
        if not self._nb_drones_seen:
            raise ParseError("missing 'nb_drones' line.")
        if not self._graph.has_start:
            raise ParseError("missing 'start_hub' zone.")
        if not self._graph.has_end:
            raise ParseError("missing 'end_hub' zone.")

    def _error(self, message: str) -> ParseError:
        """ParseError of the current line number / text.
        """
        if self._line_number == 0:
            number = None
        else:
            number = self._line_number
        return ParseError(message, number, self._raw_line)
