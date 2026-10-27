"""Reservation table of the Fly-in project.

The table remembers, for every TURN, how many drones are in each zone
and how many use each connection. The space-time A* asks it
"may I do this at that turn?", and the scheduler records the plans it
accepts, one drone after the other.
"""

from graph import Graph
from models import Zone, Connection
from plan import DronePlan, Leg


class ReservationTable:
    """Who is where, and who uses which connection, at every turn."""

    def __init__(self, graph: Graph) -> None:
        """Create an empty table for ``graph``.

        Args:
            graph: The map the drones fly in.
        """
        self._graph: Graph = graph
        self._zone_load: dict[tuple[str, int], int] = {}
        self._link_load: dict[tuple[frozenset[str], int], int] = {}

    def zone_load(self, zone: Zone, turn: int) -> int:
        """Number of drones in ``zone`` at the end of ``turn``."""
        return self._zone_load.get((zone.name, turn), 0)

    def link_load(self, connection: Connection, turn: int) -> int:
        """Number of drones using the connection a-b during ``turn``.

        Raises:
            KeyError: If the zones are not directly connected.
        """
        return self._link_load.get((connection.key, turn), 0)

    def can_stay(self, zone: Zone, turn: int) -> bool:
        """Tell whether one more drone may be in ``zone`` at ``turn``.

        Used for WAITING: the drone is already there, it asks to stay.
        """
        return zone.can_accept(self.zone_load(zone, turn))

    def can_move(
        self, origin: Zone, destination: Zone, depart_turn: int
    ) -> bool:
        """Tell whether a drone may leave ``origin`` at ``depart_turn``.

        Checks the connection during the whole move AND the capacity
        of the destination at the arrival turn.

        Args:
            origin: Zone the drone leaves.
            destination: Neighbouring zone it goes to (not blocked).
            depart_turn: Turn during which it leaves.

        Returns:
            True if no capacity would be exceeded.

        Raises:
            KeyError: If the zones are not directly connected.
            ValueError: If the destination is blocked.
        """
        link = self._graph.get_connection(origin, destination)
        arrive = depart_turn + destination.entry_cost - 1
        for turn in range(depart_turn, arrive + 1):
            if self.link_load(link, turn) >= link.max_link_capacity:
                return False
        return destination.can_accept(
            self.zone_load(destination, arrive)
        )

    def reserve_stay(self, zone: Zone, turn: int) -> None:
        """Record one drone in ``zone`` at the end of ``turn``."""
        key = (zone.name, turn)
        self._zone_load[key] = self._zone_load.get(key, 0) + 1

    def release_stay(self, zone: Zone, turn: int) -> None:
        """Remove one drone from ``zone`` at the end of ``turn``."""
        key = (zone.name, turn)
        count = self._zone_load.get(key, 0)
        if count <= 1:
            self._zone_load.pop(key, None)
        else:
            self._zone_load[key] = count - 1

    def reserve_leg(self, leg: Leg) -> None:
        """Record a move: the connection while it lasts, the arrival.

        Args:
            leg: The move to record.
        """

        for turn in range(leg.depart_turn, leg.arrive_turn + 1):
            key = (leg.connection.key, turn)
            self._link_load[key] = self._link_load.get(key, 0) + 1
        self.reserve_stay(leg.destination, leg.arrive_turn)

    def release_leg(self, leg: Leg) -> None:
        """Undo a move from the table: free the connection and the arrival."""
        for turn in range(leg.depart_turn, leg.arrive_turn + 1):
            key = (leg.connection.key, turn)
            count = self._link_load.get(key, 0)
            if count <= 1:
                self._link_load.pop(key, None)
            else:
                self._link_load[key] = count - 1
        self.release_stay(leg.destination, leg.arrive_turn)

    def reserve_plan(self, plan: DronePlan) -> None:
        """Record a whole plan: waits, moves and arrivals.

        The plan is NOT checked here: the planner (A*) is responsible
        for only building legal plans. The independent simulator
        checks them again afterwards.

        Args:
            plan: The plan of one drone.
        """
        previous_arrival = 0
        for leg in plan.legs:
            for turn in range(previous_arrival + 1, leg.depart_turn):
                self.reserve_stay(leg.origin, turn)
            self.reserve_leg(leg)
            previous_arrival = leg.arrive_turn

    def release_plan(self, plan: DronePlan) -> None:
        """Undo a whole plan from the table.

        This is the reverse operation of ``reserve_plan`` and is used by
        the optional improvement pass to rip up and re-plan selected
        drones without rebuilding the reservation table from scratch.
        """
        previous_arrival = 0
        for leg in plan.legs:
            for turn in range(previous_arrival + 1, leg.depart_turn):
                self.release_stay(leg.origin, turn)
            self.release_leg(leg)
            previous_arrival = leg.arrive_turn
