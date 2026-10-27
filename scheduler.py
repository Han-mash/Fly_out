"""Scheduler of the Fly-in project.

Turns the single-drone planner (SpaceTimeAStar) into a FLEET planner:
it plans the drones one after the other, each one around the
reservations of the previous ones, then (optionally) tries to shorten
the result.
"""

from errors import NoPathError
from graph import Graph
from pathfinding import DistanceMap
from plan import DronePlan
from reservation import ReservationTable
from space_time_a_star import SpaceTimeAStar


class Schedule:
    """The plans of the whole fleet, plus the figures derived from them.

    Passive result object: built by the Scheduler, read by the
    Simulator, the renderers and the benchmark.
    """

    def __init__(self, plans: list[DronePlan]) -> None:
        """Store the plans of the fleet.

        Args:
            plans: One plan per drone.

        Raises:
            ValueError: If ``plans`` is empty or two plans share the
                same drone id.
        """
        if not plans:
            raise ValueError("a schedule needs at least one plan")
        ids = [plan.drone_id for plan in plans]
        if len(set(ids)) != len(ids):
            raise ValueError("two plans share the same drone id")
        self.plans: list[DronePlan] = sorted(
            plans, key=lambda plan: plan.drone_id)

    @property
    def makespan(self) -> int:
        """Total number of simulation turns (= latest arrival turn)."""
        return max(plan.arrival_turn for plan in self.plans)

    @property
    def total_cost(self) -> int:
        """Sum of the movement costs of every leg of every drone.

        Waiting costs nothing here: only leg.duration is counted.
        Used as a TIE-BREAKER when two schedules have the same
        makespan.
        """
        total = 0
        for plan in self.plans:
            for leg in plan.legs:
                total += leg.duration
        return total

    def turn_line(self, turn: int) -> str:
        """Build the output line of one turn.

        Args:
            turn: Turn number (>= 1).

        Returns:
            The movements of the turn, space-separated, e.g.
            ``"D1-roof1 D2-corridorA"``. May be "" if nobody moves:
            an empty line is STILL a turn (do not skip it).
        """
        tokens: list[str] = []
        for plan in self.plans:
            token = plan.output_at(turn)
            if token is not None:
                tokens.append(token)
        return " ".join(tokens)

    def lines(self) -> list[str]:
        """Output lines of the whole simulation (one per turn).

        Returns:
            ``makespan`` lines, from turn 1 to the last arrival.
        """
        return [self.turn_line(turn) for turn in range(1, self.makespan + 1)]

    def is_better_than(self, other: "Schedule") -> bool:
        """Tell whether this schedule beats ``other``.

        Rule: fewer turns wins; on equal turns, lower total cost wins.

        Args:
            other: The schedule to compare with.

        Returns:
            True if this schedule is strictly better.
        """
        if self.makespan != other.makespan:
            return self.makespan < other.makespan
        return self.total_cost < other.total_cost


class Scheduler:
    """Plan the whole fleet on one graph.

    Strategy (prioritized planning): drone 1 gets the best plan on an
    empty table, its plan is RESERVED, drone 2 plans around it, and so
    on. Then a few rip-up and re-plan attempts try to shorten the
    result.
    """

    MAX_ROUNDS: int = 3

    def __init__(self, graph: Graph) -> None:
        """Prepare the scheduler.

        Args:
            graph: A parsed graph with a start and an end zone.

        Raises:
            NoPathError: If the end zone cannot be reached.
        """
        self._graph: Graph = graph
        self._distances: DistanceMap = DistanceMap(graph)
        start_distance = self._distances.distance(graph.start)
        if start_distance is None:
            raise NoPathError(
                "no route links the start zone to the end zone")
        self._start_distance: int = start_distance

    def build(self) -> Schedule:
        """Compute the best schedule found.

        Returns:
            A schedule for every drone of the graph.
        """
        return self._improve(self.plan_greedy())

    def plan_greedy(self) -> Schedule:
        """Plan every drone one after the other, without any retry.

        Returns:
            The greedy schedule.
        """
        table = ReservationTable(self._graph)
        planner = SpaceTimeAStar(self._graph, self._distances, table)
        plans: list[DronePlan] = []
        for number in range(1, self._graph.nb_drones + 1):
            plan = planner.find_plan(number, planner.max_turn_for(number))
            table.reserve_plan(plan)
            plans.append(plan)
        return Schedule(plans)

    def _improve(self, best: Schedule) -> Schedule:
        """Shorten a schedule by rip-up and re-plan.

        Each round removes the plan of the latest drone and the plan
        of one other drone, then plans them again in the other order.
        The first schedule that is better is kept and a new round
        starts, at most ``MAX_ROUNDS`` times.

        Args:
            best: The schedule to improve.

        Returns:
            A schedule at least as good as ``best``.
        """
        for _ in range(self.MAX_ROUNDS):
            last = max(best.plans, key=lambda plan: plan.arrival_turn)
            better = self._find_better(best, last)
            if better is None:
                break
            best = better
        return best

    def _find_better(
        self, best: Schedule, last: DronePlan
    ) -> Schedule | None:
        """Try every other drone as a partner of ``last``.

        Args:
            best: The current schedule.
            last: The plan that arrives last.

        Returns:
            The first better schedule found, or None.
        """
        for other in best.plans:
            if other.drone_id == last.drone_id:
                continue
            candidate = self._replan_pair(best, last, other)
            if candidate is not None and candidate.is_better_than(best):
                return candidate
        return None

    def _replan_pair(
        self, best: Schedule, last: DronePlan, other: DronePlan
    ) -> Schedule | None:
        """Plan two drones again, ``last`` first, around all the others.

        The table is built WITHOUT the two plans (no need to un-reserve).
        The turn limit is always enough: once the others are delivered
        (``best.makespan``), a drone flies alone in
        ``start_distance`` turns, and there are two drones to plan.

        Args:
            best: The current schedule.
            last: The first drone to plan again.
            other: The second drone to plan again.

        Returns:
            The new schedule, or None if a plan could not be found.
        """
        table = ReservationTable(self._graph)
        for plan in best.plans:
            if plan.drone_id not in (last.drone_id, other.drone_id):
                table.reserve_plan(plan)
        planner = SpaceTimeAStar(self._graph, self._distances, table)
        bound = best.makespan + 2 * self._start_distance
        try:
            new_last = planner.find_plan(last.drone_id, bound)
            table.reserve_plan(new_last)
            new_other = planner.find_plan(other.drone_id, bound)
        except NoPathError:
            return None
        kept = [
            plan for plan in best.plans
            if plan.drone_id not in (last.drone_id, other.drone_id)
        ]
        return Schedule(kept + [new_last, new_other])
