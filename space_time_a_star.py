"""Space-time A* of the Fly-in project.

Plans the route of ONE drone while respecting everything the previous
drones have already reserved. The scheduler calls it once per
drone and records each plan in the reservation table.
"""

import heapq
from dataclasses import dataclass

from errors import NoPathError
from graph import Graph
from models import Zone
from pathfinding import DistanceMap
from plan import DronePlan, Leg
from reservation import ReservationTable


@dataclass(frozen=True)
class SearchNode:
    """A state of the search: where a drone is at the END of a turn."""

    zone: str
    turn: int


@dataclass(frozen=True)
class Action:
    """One way to leave a state: wait, or fly along a leg.

    Attributes:
        zone: Zone reached.
        turn: Turn at the end of which the drone is there.
        leg: The move used, or None when the drone waits.
        penalty: Tie-break cost of this step (0 for a priority zone).
    """

    zone: Zone
    turn: int
    leg: Leg | None
    penalty: int

    @property
    def node(self) -> SearchNode:
        """The state this action leads to."""
        return SearchNode(self.zone.name, self.turn)


class Frontier:
    """Bookkeeping of one search: what to explore, what is done."""

    def __init__(self) -> None:
        """Create an empty frontier."""
        self._heap: list[tuple[int, int, int, int, str, int]] = []
        self._parents: dict[SearchNode, tuple[SearchNode, Leg | None]] = {}
        self._best: dict[SearchNode, int] = {}
        self._closed: set[SearchNode] = set()
        self._sequence: int = 0

    def start(self, zone: str, distance: int) -> None:
        """Put the start state (turn 0) on the frontier.

        Args:
            zone: Name of the start zone.
            distance: Heuristic of the start zone.
        """
        node = SearchNode(zone, 0)
        self._best[node] = 0
        self._push(node, 0, distance)

    def push(
        self, action: Action, parent: SearchNode,
        penalty: int, distance: int,
    ) -> None:
        """Offer the state reached by ``action``.

        It is ignored if it is already done, or if a path with a penalty
        at least as good is already known.

        Args:
            action: The step that reaches the state.
            parent: The state the step starts from.
            penalty: Total tie-break penalty of the path.
            distance: Heuristic of the zone reached.
        """
        node = action.node
        if node in self._closed:
            return
        if node in self._best and self._best[node] <= penalty:
            return
        self._best[node] = penalty
        self._parents[node] = (parent, action.leg)
        self._push(node, penalty, distance)

    def pop(self) -> SearchNode | None:
        """Take the most promising state not yet explored.

        Returns:
            The state (now marked as done), or None if the frontier is
            empty.
        """
        while self._heap:
            _, _, _, _, zone, turn = heapq.heappop(self._heap)
            node = SearchNode(zone, turn)
            if node not in self._closed:
                self._closed.add(node)
                return node
        return None

    def penalty(self, node: SearchNode) -> int:
        """Best tie-break penalty found for ``node``."""
        return self._best[node]

    def legs_to(self, goal: SearchNode) -> list[Leg]:
        """Walk back from ``goal`` and list the legs in order.

        Args:
            goal: A state reached by the search.

        Returns:
            The legs from the start to ``goal``, oldest first. Waits
            produce no leg.
        """
        legs: list[Leg] = []
        node = goal
        while node in self._parents:
            node, leg = self._parents[node]
            if leg is not None:
                legs.append(leg)
        legs.reverse()
        return legs

    def _push(self, node: SearchNode, penalty: int, distance: int) -> None:
        """Add a heap item; the sequence number keeps the order stable."""
        self._sequence = self._sequence + 1
        heapq.heappush(self._heap, (
            node.turn + distance, penalty, distance,
            self._sequence, node.zone, node.turn))


class SpaceTimeAStar:
    """A* in the (zone, turn) space for a single drone."""

    def __init__(
        self,
        graph: Graph,
        distances: DistanceMap,
        table: ReservationTable,
    ) -> None:
        """Create a planner.

        Args:
            graph: The map.
            distances: Distances to the end zone (the heuristic).
            table: Reservations made by the drones already planned.
        """
        self._graph: Graph = graph
        self._distances: DistanceMap = distances
        self._table: ReservationTable = table

    def max_turn_for(self, rank: int) -> int:
        """Last turn a search must reach for the ``rank``-th drone.

        The k-th drone can always wait on the start zone (no limit
        there) until the previous drones are delivered, then fly alone
        along the shortest route: a plan exists within
        ``rank * distance(start)`` turns.

        Args:
            rank: Position of the drone in the planning order (>= 1).

        Returns:
            A turn limit that always allows a plan.

        Raises:
            ValueError: If rank < 1.
            NoPathError: If the end zone cannot be reached.
        """
        if rank < 1:
            raise ValueError(f"rank must be >= 1, got {rank}")
        start_distance = self._distances.distance(self._graph.start)
        if start_distance is None:
            raise NoPathError("the end zone cannot be reached")
        return rank * start_distance

    def find_plan(self, drone_id: int, max_turn: int) -> DronePlan:
        """Find the fastest legal plan of one drone.

        Args:
            drone_id: Identifier of the drone (>= 1).
            max_turn: Last turn the search may reach.

        Returns:
            A plan from the start zone to the end zone.

        Raises:
            NoPathError: If the end zone cannot be reached from the
                start, or not within ``max_turn`` turns.
        """
        start, end = self._graph.start, self._graph.end
        start_distance = self._distances.distance(start)
        if start_distance is None:
            raise NoPathError("the end zone cannot be reached")
        if start_distance > max_turn:
            raise NoPathError("max_turn is shorter than the route")
        frontier = Frontier()
        frontier.start(start.name, start_distance)
        node = frontier.pop()
        while node is not None:
            if self._graph.get_zone(node.zone) == end:
                return DronePlan(drone_id, frontier.legs_to(node))
            self._expand(frontier, node, max_turn)
            node = frontier.pop()
        raise NoPathError(f"no plan within {max_turn} turns")

    def _expand(
        self, frontier: Frontier, node: SearchNode, max_turn: int
    ) -> None:
        """Offer to the frontier every state reachable from ``node``.

        Args:
            frontier: The search bookkeeping.
            node: The state being explored.
            max_turn: Last turn the search may reach.
        """
        zone = self._graph.get_zone(node.zone)
        penalty = frontier.penalty(node)
        for action in self._actions(zone, node.turn):
            distance = self._distances.distance(action.zone)
            if distance is None or action.turn + distance > max_turn:
                continue
            frontier.push(action, node, penalty + action.penalty, distance)

    def _actions(self, zone: Zone, turn: int) -> list[Action]:
        """List what a drone may do from ``zone`` at the end of ``turn``.

        Args:
            zone: Where the drone is.
            turn: The turn at the end of which it is there.

        Returns:
            One Action per legal choice (waiting, or moving).
        """
        actions: list[Action] = []
        if self._table.can_stay(zone, turn + 1):
            actions.append(Action(zone, turn + 1, None, 1))
        depart = turn + 1
        for neighbour in self._graph.neighbors(zone):
            if self._table.can_move(zone, neighbour, depart):
                link = self._graph.get_connection(zone, neighbour)
                leg = Leg(zone, link, depart)
                penalty = 0 if neighbour.is_priority else neighbour.entry_cost
                actions.append(
                    Action(neighbour, leg.arrive_turn, leg, penalty))
        return actions
