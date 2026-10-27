"""Data models for the Fly-in project.

Classes in this file: ZoneType, Zone, Connection, Drone.

They are PASSIVE data holders: they describe the world but contain no
search algorithm and no parsing logic.

"""

from enum import Enum


class ZoneType(Enum):
    """The four kinds of zones allowed by the subject.

    Values are the exact strings used in map files (``zone=<type>``).
    """

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"

    @classmethod
    def from_string(cls, value: str) -> "ZoneType":
        """Convert a map-file string into a ZoneType.

        Args:
            value: Text found after ``zone=`` in a map file.

        Returns:
            The matching ZoneType.

        Raises:
            ValueError: If the text is not one of the four types.
        """
        for member in cls:
            if member.value == value:
                return member
        expected = ", ".join(member.value for member in cls)
        raise ValueError(
            f"Unknown zone type {value!r} (expected one of: {expected})"
        )

    @property
    def is_blocked(self) -> bool:
        """Tell whether drones are forbidden to enter this type."""
        return self is ZoneType.BLOCKED

    @property
    def is_restricted(self) -> bool:
        """Tell whether entering costs 2 turns (drone in flight)."""
        return self is ZoneType.RESTRICTED

    @property
    def is_priority(self) -> bool:
        """Tell whether the pathfinding should prefer this type."""
        return self is ZoneType.PRIORITY

    @property
    def entry_cost(self) -> int:
        """Number of turns needed to ENTER a zone of this type.

        Returns:
            1 for normal and priority, 2 for restricted.

        Raises:
            ValueError: If the type is blocked (no valid cost).
        """
        if self is ZoneType.BLOCKED:
            raise ValueError("Error, a blocked zone has no entry cost")
        if self is self.RESTRICTED:
            return 2
        return 1


class Zone:
    """A node of the graph (start, end or regular hub).

    """

    FORBIDDEN_NAME_CHARS: frozenset[str] = frozenset({"-", "#", "[", "]"})

    @staticmethod
    def is_valid_name(name: str) -> bool:
        """Tell whether a string can be used as a zone name.

        Args:
            name: Candidate name.

        Returns:
            True if the name is non-empty and has no whitespace and
            no forbidden character.
        """
        if name == "":
            return False
        for character in name:
            if character.isspace() or character in Zone.FORBIDDEN_NAME_CHARS:
                return False
        return True

    def __init__(
        self,
        name: str,
        x: int,
        y: int,
        zone_type: ZoneType = ZoneType.NORMAL,
        color: str | None = None,
        max_drones: int = 1,
        is_start: bool = False,
        is_end: bool = False,
    ) -> None:
        """Create a zone.

        Args:
            name: Unique name (no dash, no space).
            x: Integer x coordinate (can be negative).
            y: Integer y coordinate (can be negative).
            zone_type: Kind of zone.
            color: Optional single-word colour (any word is valid).
            max_drones: Declared capacity (positive integer).
            is_start: True if this is the start hub.
            is_end: True if this is the end hub.

        Raises:
            ValueError: If name is invalid, max_drones < 1, or the
                zone is declared both start and end.
        """
        if not Zone.is_valid_name(name):
            raise ValueError(f"Invalid data on {name!r}")
        if max_drones < 1:
            raise ValueError(f"Invalid data on {max_drones!r}")
        if is_start and is_end:
            raise ValueError(f"Invalid data on {is_start!r}, {is_end!r}")
        self.name: str = name
        self.x: int = x
        self.y: int = y
        self.zone_type: ZoneType = zone_type
        self.color: str | None = color
        self.max_drones: int = max_drones
        self.is_start: bool = is_start
        self.is_end: bool = is_end

    @property
    def capacity(self) -> int | None:
        """Effective capacity of the zone.

        Returns:
            None when unlimited (start and end zones), otherwise
            the value of max_drones.
        """
        if self.is_start or self.is_end:
            return None
        return self.max_drones

    def can_accept(self, occupants: int) -> bool:
        """Tell whether one more drone fits in this zone.

        Args:
            occupants: Number of drones already counted in the zone.

        Returns:
            True if one more drone would not exceed the capacity.
        """
        limit = self.capacity
        if limit is None:
            return True
        return occupants < limit

    @property
    def is_blocked(self) -> bool:
        """Shortcut: is this zone forbidden to drones?"""
        return self.zone_type.is_blocked

    @property
    def is_restricted(self) -> bool:
        """Shortcut: does entering this zone take 2 turns?"""
        return self.zone_type.is_restricted

    @property
    def is_priority(self) -> bool:
        """Shortcut: is this a preferred zone?"""
        return self.zone_type.is_priority

    @property
    def entry_cost(self) -> int:
        """Turns needed to enter this zone (delegates to its type)."""
        if self.is_blocked:
            raise ValueError("Error, Zone blocked")
        return self.zone_type.entry_cost

    def __eq__(self, other: object) -> bool:
        """Two zones are equal if they have the same name."""
        if not isinstance(other, Zone):
            return NotImplemented
        return self.name == other.name

    def __hash__(self) -> int:
        """Hash consistent with __eq__ (needed for dict / set keys)."""
        return hash(self.name)

    def __repr__(self) -> str:
        """Debug representation of the zone."""
        return (
            f"Zone({self.name!r}, x={self.x}, y={self.y}, "
            f"type={self.zone_type.value})"
        )


class Connection:
    """A bidirectional edge between two zones."""

    def __init__(
        self,
        zone_a: Zone,
        zone_b: Zone,
        max_link_capacity: int = 1,
    ) -> None:
        """Create a connection.

        Args:
            zone_a: One end of the connection.
            zone_b: The other end.
            max_link_capacity: Max drones using it simultaneously.

        Raises:
            ValueError: If capacity < 1 or if both ends are the same
                zone.
        """
        if max_link_capacity < 1:
            raise ValueError("Error, Capacity must be equal or more than 1.")
        if zone_a == zone_b:
            raise ValueError("Error, Both ends are the same zone.")
        self.zone_a: Zone = zone_a
        self.zone_b: Zone = zone_b
        self.max_link_capacity: int = max_link_capacity
        self._key: frozenset[str] = Connection.make_key(
            zone_a.name, zone_b.name
        )

    @staticmethod
    def make_key(name_a: str, name_b: str) -> frozenset[str]:
        """Build the canonical key of a connection.

        ``a-b`` and ``b-a`` must give the SAME key.

        Args:
            name_a: Name of the first zone.
            name_b: Name of the second zone.

        Returns:
            An order-independent key usable in a dict.
        """
        return frozenset({name_a, name_b})

    @property
    def key(self) -> frozenset[str]:
        """Canonical key of this connection."""
        return self._key

    def connects(self, zone: Zone) -> bool:
        """Tell whether the zone is one of the two ends."""
        return zone == self.zone_a or zone == self.zone_b

    def other_end(self, zone: Zone) -> Zone:
        """Return the zone at the opposite end.

        Args:
            zone: One end of the connection.

        Returns:
            The other end.

        Raises:
            ValueError: If the zone is not an end of this connection.
        """
        if zone == self.zone_a:
            return self.zone_b
        if zone == self.zone_b:
            return self.zone_a
        raise ValueError("Zone X is not an end of this connection.")

    def label_from(self, origin: Zone) -> str:
        """Name used in the output for a drone in flight."""
        try:
            if origin not in (self.zone_a, self.zone_b):
                raise ValueError(f"{origin.name} not an end's connection.")
            dest = self.other_end(origin)
            return f"{origin.name}-{dest.name}"
        except ValueError as e:
            raise ValueError(
                f"{origin.name} is not an end of this connection.") from e

    def __repr__(self) -> str:
        """Debug representation of the connection."""
        a = f"{self.zone_a.name}"
        b = f"{self.zone_b.name}"
        c = f"{self.max_link_capacity}"
        return f"Connection({a}, {b}, cap={c})"


class Drone:
    """A drone of the fleet.

    Only holds an identifier and a position. The routes live in
    DronePlan and the in-flight state lives in the Simulator.
    """

    def __init__(self, drone_id: int, start: Zone) -> None:
        """Create a drone placed on the start zone.

        Args:
            drone_id: Positive integer identifier (1, 2, 3...).
            start: The start zone.

        Raises:
            ValueError: If drone_id < 1.
        """
        if drone_id < 1:
            raise ValueError("Drone_id must be integer identifier (1, 2...)")
        self.drone_id: int = drone_id
        self.current_zone: Zone = start

    @property
    def name(self) -> str:
        """Identifier printed in the output, e.g. ``"D1"``."""
        return f"D{self.drone_id}"

    def __repr__(self) -> str:
        """Debug representation of the drone."""
        return f"Drone({self.name}, at={self.current_zone.name})"
