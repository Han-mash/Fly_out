
"""Independent referee for the Fly-in project.

Reads an output line by line, simulates every drone movement, and raises
SimulationError on the first violated rule.
"""

import re
from collections import Counter
from dataclasses import dataclass

from errors import SimulationError
from graph import Graph
from models import Connection, Zone


TOKEN = re.compile(r"D([1-9]\d*)-(\S+)")


@dataclass(frozen=True)
class Move:
    """One movement token from an output line."""

    drone_id: int
    target: str
    is_flight: bool


@dataclass(frozen=True)
class Flight:
    """A drone travelling through a connection toward a restricted zone."""

    connection: Connection
    destination: Zone


@dataclass(frozen=True)
class TurnState:
    """Snapshot of the fleet at the end of one turn."""

    turn: int
    positions: dict[int, str]
    moves: int


@dataclass(frozen=True)
class SimulationResult:
    """Metrics produced by a successful simulation."""

    turns: int
    history: list[TurnState]
    arrival_turns: dict[int, int]
    total_cost: int

    @property
    def average_turns(self) -> float:
        """Average delivery turn across all drones."""
        if not self.arrival_turns:
            return 0.0

        return sum(self.arrival_turns.values()) / len(self.arrival_turns)

    @property
    def moves_per_turn(self) -> list[int]:
        """Number of movement tokens written on each turn."""
        return [state.moves for state in self.history]


@dataclass
class DroneState:
    """Current state of a drone."""

    zone: Zone
    flight: Flight | None = None
    delivered: bool = False

    @property
    def in_flight(self) -> bool:
        """Whether the drone is currently travelling on a connection."""
        return self.flight is not None


class Simulator:
    """Simulate an output and enforce the map's movement rules."""

    def __init__(self, graph: Graph) -> None:
        """Prepare a referee for one graph."""
        self._graph: Graph = graph
        self._states: dict[int, DroneState] = {}
        self._links: dict[frozenset[str], int] = {}
        self._history: list[TurnState] = []
        self._arrivals: dict[int, int] = {}
        self._cost: int = 0

        self._reset()

    def parse_line(self, turn: int, line: str) -> list[Move]:
        """Parse and validate the syntax of a single output line."""
        moves: list[Move] = []

        for token in line.split():
            found = TOKEN.fullmatch(token)

            if found is None or found.group(2).count("-") > 1:
                raise SimulationError(
                    f"turn {turn}: invalid token {token!r}"
                )

            drone_id = int(found.group(1))
            target = found.group(2)

            moves.append(
                Move(
                    drone_id=drone_id,
                    target=target,
                    is_flight="-" in target,
                )
            )

        return moves

    def run(self, lines: list[str]) -> SimulationResult:
        """Simulate all output lines and return the resulting metrics."""
        self._reset()

        for turn, line in enumerate(lines, start=1):
            if all(state.delivered for state in self._states.values()):
                raise SimulationError(
                    f"turn {turn}: every drone is already delivered"
                )

            moves = self.parse_line(turn, line)
            self._play_turn(turn, moves)

        for drone_id, state in self._states.items():
            if not state.delivered:
                raise SimulationError(
                    f"D{drone_id} never reaches the end zone"
                )

        return SimulationResult(
            turns=len(lines),
            history=list(self._history),
            arrival_turns=dict(self._arrivals),
            total_cost=self._cost,
        )

    def _reset(self) -> None:
        """Restore the initial fleet state before each simulation."""
        start = self._graph.start

        self._states = {
            number: DroneState(start)
            for number in range(1, self._graph.nb_drones + 1)
        }

        self._links = {}
        self._history = []
        self._arrivals = {}
        self._cost = 0

    def _index(self, turn: int, moves: list[Move]) -> dict[int, Move]:
        """Index moves by drone and reject unknown or repeated drones."""
        by_drone: dict[int, Move] = {}

        for move in moves:
            if move.drone_id not in self._states:
                raise SimulationError(
                    f"turn {turn}: unknown drone D{move.drone_id}"
                )

            if move.drone_id in by_drone:
                raise SimulationError(
                    f"turn {turn}: drone D{move.drone_id} moves more than once"
                )

            by_drone[move.drone_id] = move

        return by_drone

    def _play_turn(self, turn: int, moves: list[Move]) -> None:
        """Apply mandatory landings, new movements, then capacity checks."""
        by_drone = self._index(turn, moves)
        self._links = {}

        for drone_id, state in self._states.items():
            if state.flight is not None:
                move = by_drone.pop(drone_id, None)
                self._land(turn, drone_id, state, move)

        for drone_id, move in by_drone.items():
            self._depart(turn, drone_id, move)

        self._check_capacities(turn)
        self._snapshot(turn, len(moves))

    def _land(
        self,
        turn: int,
        drone_id: int,
        state: DroneState,
        move: Move | None,
    ) -> None:
        """Complete a flight; the drone must land at its destination."""
        flight = state.flight

        if flight is None:
            return

        if (
            move is None
            or move.is_flight
            or move.target != flight.destination.name
        ):
            raise SimulationError(
                f"turn {turn}: D{drone_id} is in flight toward "
                f"{flight.destination.name!r} and must arrive now"
            )

        self._use_link(turn, flight.connection)
        self._arrive(turn, drone_id, flight.destination)

    def _arrive(self, turn: int, drone_id: int, zone: Zone) -> None:
        """Place a drone in a zone and record delivery when applicable."""
        state = self._states[drone_id]
        state.zone = zone
        state.flight = None

        if zone.is_end:
            state.delivered = True
            self._arrivals[drone_id] = turn

    def _depart(self, turn: int, drone_id: int, move: Move) -> None:
        """Start a normal movement or a two-turn flight."""
        state = self._states[drone_id]

        if state.delivered:
            raise SimulationError(
                f"turn {turn}: D{drone_id} is already delivered"
            )

        origin = state.zone
        destination = self._target_zone(
            turn, drone_id, origin, move
        )
        connection = self._graph.get_connection(
            origin, destination
        )

        self._use_link(turn, connection)
        self._cost += destination.entry_cost

        if move.is_flight:
            state.flight = Flight(connection, destination)
        else:
            self._arrive(turn, drone_id, destination)

    def _target_zone(
        self,
        turn: int,
        drone_id: int,
        origin: Zone,
        move: Move,
    ) -> Zone:
        """Validate the destination, connection, and movement type."""
        name = move.target

        if move.is_flight:
            first, name = name.split("-", 1)

            if first != origin.name:
                raise SimulationError(
                    f"turn {turn}: D{drone_id} cannot start flight "
                    f"from {first!r}; its current zone is {origin.name!r}"
                )

        if not self._graph.has_zone(name):
            raise SimulationError(
                f"turn {turn}: D{drone_id} targets unknown zone {name!r}"
            )

        zone = self._graph.get_zone(name)

        if zone.is_blocked:
            raise SimulationError(
                f"turn {turn}: D{drone_id} cannot enter blocked zone {name!r}"
            )

        if not self._graph.has_connection(origin.name, name):
            raise SimulationError(
                f"turn {turn}: {origin.name!r} and {name!r} "
                "are not connected"
            )

        if zone.is_restricted and not move.is_flight:
            raise SimulationError(
                f"turn {turn}: zone {name!r} is restricted; "
                "write the connection first"
            )

        if move.is_flight and not zone.is_restricted:
            raise SimulationError(
                f"turn {turn}: zone {name!r} is not restricted; "
                "write the zone name instead"
            )

        return zone

    def _use_link(self, turn: int, link: Connection) -> None:
        """Count a connection's use in either direction during this turn."""
        used = self._links.get(link.key, 0) + 1
        self._links[link.key] = used

        if used > link.max_link_capacity:
            raise SimulationError(
                f"turn {turn}: connection capacity exceeded "
                f"({used} drones, maximum {link.max_link_capacity})"
            )

    def _check_capacities(self, turn: int) -> None:
        """Check the occupancy of each zone at the end of the turn."""
        counts: Counter[str] = Counter()

        for state in self._states.values():
            if not state.delivered and state.flight is None:
                counts[state.zone.name] += 1

        for name, count in counts.items():
            capacity = self._graph.get_zone(name).capacity

            if capacity is not None and count > capacity:
                raise SimulationError(
                    f"turn {turn}: zone {name!r} holds {count} drones "
                    f"(maximum {capacity})"
                )

    def _snapshot(self, turn: int, moves: int) -> None:
        """Save the end-of-turn positions for the renderer."""
        positions: dict[int, str] = {}

        for drone_id, state in self._states.items():
            if state.flight is not None:
                origin = state.zone.name
                destination = state.flight.destination.name
                positions[drone_id] = f"{origin}-{destination}"
            else:
                positions[drone_id] = state.zone.name

        self._history.append(
            TurnState(
                turn=turn,
                positions=positions,
                moves=moves,
            )
        )
