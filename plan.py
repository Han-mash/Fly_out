"""Plans of the Fly-in project: what ONE drone does, turn by turn.
"""

from models import Connection, Zone


class Leg:
    """One move of a drone between two neighbouring zone."""

    def __init__(
            self,
            origin: Zone,
            connection: Connection,
            depart_turn: int
            ) -> None:
        """Creating leg.

        Args:
            origin: Zone the drone leaves.
            connection: Connection it travels along.
            depart_turn: Turn (>= 1) during which
            the drone leaves.

        Raises:
            ValueError: If depart_turn < 1, if ``origin`` is
            not an end of the connection, or if the destination
            is a blocked zone.
            """
        if depart_turn < 1:
            raise ValueError(f"Depart_turn must be >= 1, got {depart_turn}.")
        if not connection.connects(origin):
            raise ValueError("Error.")
        if connection.other_end(origin).is_blocked:
            raise ValueError("Error.")
        self.origin: Zone = origin
        self.connection: Connection = connection
        self.depart_turn: int = depart_turn

    @property
    def destination(self) -> Zone:
        """Zone the drone reaches."""
        return self.connection.other_end(self.origin)

    @property
    def duration(self) -> int:
        """Turns needed for this move(1, or 2 for a restricted zone)."""
        return self.destination.entry_cost

    @property
    def arrive_turn(self) -> int:
        """Turn at the END of which the drone is in the destination."""
        return self.depart_turn + self.duration - 1

    @property
    def is_flight(self) -> bool:
        """Tell whether the drone is in flight during depart_turn."""
        return self.duration > 1

    def output_at(self, turn: int) -> str | None:
        """Text printed for this leg during ``turn``.

        Args:
            turn: A turn number.

        Returns:
            the destination name, or the connection name while the
            drone is in flight, or None if the legs prints nothing at
            this turn
        """
        if turn == self.depart_turn:
            if self.is_flight:
                return self.connection.label_from(self.origin)
            return self.destination.name
        if self.is_flight and turn == self.arrive_turn:
            return self.destination.name
        return None

    def __repr__(self) -> str:
        """Debug representation of the leg"""
        msg: str = f" turns {self.depart_turn}...{self.arrive_turn}"
        return f"Leg({self.origin} -> {self.destination}, {msg})"


class DronePlan:
    """The complete schedule of one drone, from start to end zone."""
    def __init__(self, drone_id: int, legs: list[Leg]) -> None:
        """Creating plan.

        Args:
            drone_id: Positive drone identifier.
            legs: the moves, in chronological order.

        Raises:
            ValueError: If drone_id < 1, if there is no leg, or if two
            consecutive legs are not chained (the next one must
            START where the previous one ENDED, and DEPART
            strictly AFTER its arrival turn)."""
        if drone_id < 1:
            raise ValueError("Drones_id must be positive.")
        if not legs:
            raise ValueError("A plan needs a leg.")
        for prev, follow in zip(legs, legs[1:]):
            if follow.origin != prev.destination:
                raise ValueError("legs are not chained.")
            if follow.depart_turn <= prev.arrive_turn:
                raise ValueError(
                    "A drone cannot leave before it has arrived."
                )
        self.drone_id: int = drone_id
        self.legs: list[Leg] = list(legs)

    @property
    def name(self) -> str:
        """Drone name as printed in the output."""
        return f"D{self.drone_id}"

    @property
    def arrival_turn(self) -> int:
        """Turn at which drone reaches its last zone."""
        return self.legs[-1].arrive_turn

    def position_at(self, turn: int) -> Zone | None:
        """Zone where the drone is at the END of ``turn``.

        Args:
            turn: A turn number (0=before anything happens).

        Returns:
            The zone, or None while the drone is in flight on a
            connection. After the last arrival, the last zone.

        Raises:
            ValueError: if turn < 0.
        """
        if turn < 0:
            raise ValueError("Error")
        position = self.legs[0].origin
        for leg in self.legs:
            if turn < leg.depart_turn:
                return position
            if turn < leg.arrive_turn:
                return None
            position = leg.destination
        return position

    def output_at(self, turn: int) -> str | None:
        """Output token of this drone at ``turn``.

        Returns:
            The token, or None if the drone prints nothing this turn
            (waiting, or already delivered).
        """
        for leg in self.legs:
            text = leg.output_at(turn)
            if text is not None:
                return f"{self.name}-{text}"
        return None

    def __repr__(self) -> str:
        """Debug representation of the plan."""
        return f"DronePlan({self.name}, {self.legs})"
