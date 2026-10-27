"""Pathfinding tools of the Fly-in project.
"""

import heapq
from graph import Graph
from models import Zone


class DistanceMap:
    """Minimum number of turns from every zone to the end zone.
    """
    def __init__(self, graph: Graph) -> None:
        """Compute the distances to the end zone of ``graph``.

        Args:
            graph: A graph with a start and an end zone.

        Raises:
            ValueError: if the graph has no end zone.
        """
        self._graph: Graph = graph
        self._distances: dict[str, int] = {}
        self._compute()

    def _compute(self) -> None:
        """Fill self._distances with a reverse Dijkstra."""
        goal = self._graph.end
        if goal.is_blocked:
            return
        distances = {goal.name: 0}
        heap = [(0, goal.name)]
        while heap:
            d, name = heapq.heappop(heap)
            if d > distances[name]:
                continue
            current = self._graph.get_zone(name)
            step = current.entry_cost
            for n in self._graph.neighbors(current):
                candidate = d + step
                if n.name not in distances or candidate < distances[n.name]:
                    distances[n.name] = candidate
                    heapq.heappush(heap, (candidate, n.name))
        self._distances = distances

    def distance(self, zone: Zone) -> int | None:
        """Minimum number of turns from ``zone`` to the end zone.

        Args:
            zone: Any zone of the graph.

        Returns:
            The distance in turns, or None if the end zone cannot be
            reached from ``zone``.
        """
        return self._distances.get(zone.name)

    def is_reachable(self, zone: Zone) -> bool:
        """Tell whether the end zone can be reached from ``zone``."""
        return zone.name in self._distances
