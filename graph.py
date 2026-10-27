"""Graph structure of the Fly-in project.

The graph only answers questions about its own structure. It contains
NO routing algorithm.
"""

from collections import deque
from models import Connection, Zone


class Graph:
    """A network of zones linked by bidirectional connections.
    """

    def __init__(self) -> None:
        """Create an empty graph."""
        self._zones: dict[str, Zone] = {}
        self._connections: dict[frozenset[str], Connection] = {}
        self._adjacency: dict[str, list[Zone]] = {}
        self._start: Zone | None = None
        self._end: Zone | None = None
        self._nb_drones: int = 0

    def set_nb_drones(self, count: int) -> None:
        """Set the size of the fleet.

        Args:
            count: Number of drones.

        Raises:
            ValueError: If count is not a positive integer.
        """
        if count < 1:
            raise ValueError(f"nb_drones must be >= 1, got {count}")
        self._nb_drones = count

    @property
    def nb_drones(self) -> int:
        """Number of drones in the fleet."""
        return self._nb_drones

    def add_zone(self, zone: Zone) -> None:
        """Register a zone; remember it if it is the start or the end.

        Args:
            zone: The zone to add.

        Raises:
            ValueError: If the name is already used, or if a second
                start / end zone is added.
        """
        if zone.name in self._zones:
            raise ValueError("zone name already used.")
        if zone.is_start and self._start is not None:
            raise ValueError("a start zone already exists.")
        if zone.is_end and self._end is not None:
            raise ValueError("an end zone already exists.")
        self._zones[zone.name] = zone
        self._adjacency[zone.name] = []
        if zone.is_start:
            self._start = zone
        if zone.is_end:
            self._end = zone

    def add_connection(self, connection: Connection) -> None:
        """Register a connection and update the adjacency.

        Args:
            connection: The connection to add.

        Raises:
            ValueError: If the same connection already exists
                (in any direction) or if an end is unknown.
        """
        name_a = connection.zone_a.name
        name_b = connection.zone_b.name
        if name_a not in self._zones or name_b not in self._zones:
            raise ValueError("unknown zone in connection.")
        if connection.key in self._connections:
            raise ValueError("duplicate connection.")
        zone_a = self._zones[name_a]
        zone_b = self._zones[name_b]
        self._connections[connection.key] = connection
        self._adjacency[name_a].append(zone_b)
        self._adjacency[name_b].append(zone_a)

    def has_zone(self, name: str) -> bool:
        """Tell whether a zone with this name exists."""
        return name in self._zones

    def get_zone(self, name: str) -> Zone:
        """Return the zone with this name.

        Args:
            name: Name of the zone.

        Returns:
            The zone.

        Raises:
            KeyError: If the zone does not exist.
        """
        return self._zones[name]

    def has_connection(self, name_a: str, name_b: str) -> bool:
        """Tell whether a connection exists, whatever the direction."""
        key = Connection.make_key(name_a, name_b)
        return key in self._connections

    def get_connection(self, zone_a: Zone, zone_b: Zone) -> Connection:
        """Return the connection between two zones (any direction).

        Args:
            zone_a: One zone.
            zone_b: The other zone.

        Returns:
            The connection linking them.

        Raises:
            KeyError: If the two zones are not directly connected.
        """
        key = Connection.make_key(zone_a.name, zone_b.name)
        return self._connections[key]

    def neighbors(self, zone: Zone) -> list[Zone]:
        """Return the zones a drone can move to from ``zone``.

        Blocked zones are NOT returned.

        Args:
            zone: The zone the drone is on.

        Returns:
            The reachable neighbours, in definition order.

        Raises:
            KeyError: If the zone is not in the graph.
        """
        return [
            other for other in self._adjacency[zone.name]
            if not other.is_blocked
        ]

    @property
    def zones(self) -> list[Zone]:
        """All the zones, in definition order."""
        return list(self._zones.values())

    @property
    def connections(self) -> list[Connection]:
        """All the connections, in definition order."""
        return list(self._connections.values())

    @property
    def has_start(self) -> bool:
        """Tell whether a start zone was defined."""
        return self._start is not None

    @property
    def has_end(self) -> bool:
        """Tell whether an end zone was defined."""
        return self._end is not None

    @property
    def start(self) -> Zone:
        """The start zone.

        Raises:
            ValueError: If no start zone was defined.
        """
        if self._start is None:
            raise ValueError("no start zone defined.")
        return self._start

    @property
    def end(self) -> Zone:
        """The end zone.

        Raises:
            ValueError: If no end zone was defined.
        """

        if self._end is None:
            raise ValueError("no end zone defined")
        return self._end

    def bounds(self) -> tuple[int, int, int, int]:
        """Bounding box of the map, for the renderers.

        Returns:
            (min_x, min_y, max_x, max_y) over all the zones.

        Raises:
            ValueError: If the graph has no zone.
        """
        if not self._zones:
            raise ValueError("empty graph")
        xs = [zone.x for zone in self._zones.values()]
        ys = [zone.y for zone in self._zones.values()]
        return (min(xs), min(ys), max(xs), max(ys))

    def is_goal_reachable(self) -> bool:
        """Tell whether the end zone can be reached from the start.

        Returns:
            True if a route exists, False otherwise.

        Raises:
            ValueError: If the start or the end zone is missing.
        """
        source = self.start
        target = self.end
        if source.is_blocked or target.is_blocked:
            return False
        queue = deque([source])
        visited = {source}
        while queue:
            current = queue.popleft()
            if current == target:
                return True
            for neighbour in self.neighbors(current):
                if neighbour not in visited:
                    visited.add(neighbour)
                    queue.append(neighbour)
        return False
