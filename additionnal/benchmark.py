"""Benchmark of the Fly-in project: run every map and compare the turns.

Usage:
    python3 benchmark.py            run all the maps of ./maps
    python3 benchmark.py DIRECTORY  run all the maps of another folder

The reference values come from the evaluation sheet. The script exits
with 1 when a map needs more turns than its reference.
"""

import sys
import time
from dataclasses import dataclass
from pathlib import Path

from errors import FlyInError
from map_parser import MapParser
from scheduler import Scheduler
from simulator import Simulator


REFERENCES: dict[str, int] = {
    "01_linear_path": 6,
    "02_simple_fork": 6,
    "03_basic_capacity": 8,
    "01_dead_end_trap": 15,
    "02_circular_loop": 20,
    "03_priority_puzzle": 12,
    "01_maze_nightmare": 45,
    "02_capacity_hell": 60,
    "03_ultimate_challenge": 35,
    "01_the_impossible_dream": 45,
}


@dataclass(frozen=True)
class BenchResult:
    """Figures measured on one map."""

    name: str
    drones: int
    turns: int
    seconds: float
    reference: int | None

    @property
    def passed(self) -> bool:
        """Tell whether the turns are within the reference (if any)."""
        return self.reference is None or self.turns <= self.reference


class Benchmark:
    """Run the planner and the referee on a list of maps."""

    def __init__(self, folder: Path) -> None:
        """Remember the folder that holds the maps."""
        self._folder: Path = folder

    def run(self) -> list[BenchResult]:
        """Solve every map found in the folder, sorted by path."""
        results: list[BenchResult] = []
        for path in sorted(self._folder.rglob("*.txt")):
            results.append(self._solve(path))
        return results

    def _solve(self, path: Path) -> BenchResult:
        """Parse, plan and check one map, and time the planning."""
        graph = MapParser().parse_file(str(path))
        begin = time.perf_counter()
        schedule = Scheduler(graph).build()
        seconds = time.perf_counter() - begin
        turns = Simulator(graph).run(schedule.lines()).turns
        return BenchResult(path.stem, graph.nb_drones, turns, seconds,
                           REFERENCES.get(path.stem))

    @staticmethod
    def show(results: list[BenchResult]) -> None:
        """Print the table of the results."""
        print(f"{'map':26} {'drones':>6} {'turns':>6} {'ref':>5} "
              f"{'time':>7}  verdict")
        for item in results:
            reference = "-" if item.reference is None else item.reference
            verdict = "ok" if item.passed else "TOO SLOW"
            print(f"{item.name:26} {item.drones:6} {item.turns:6} "
                  f"{reference!s:>5} {item.seconds:6.2f}s  {verdict}")


def main() -> int:
    """Entry point: return 1 if one map is over its reference."""
    folder = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("maps")
    try:
        results = Benchmark(folder).run()
    except FlyInError as error:
        print(error, file=sys.stderr)
        return 1
    Benchmark.show(results)
    return 0 if all(item.passed for item in results) else 1


if __name__ == "__main__":
    sys.exit(main())
