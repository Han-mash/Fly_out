import argparse
import sys
from typing import TextIO

from errors import DisplayError, FlyInError
from graph import Graph
from map_parser import MapParser
from scheduler import Schedule, Scheduler
from simulator import SimulationResult, Simulator


class App:
    """The program: read a map, plan, check, print, show."""

    def __init__(
        self,
        out: TextIO | None = None,
        err: TextIO | None = None,
        max_frames: int | None = None,
    ) -> None:
        """Create the program."""
        self._out: TextIO = out if out is not None else sys.stdout
        self._err: TextIO = err if err is not None else sys.stderr
        self._max_frames: int | None = max_frames

    def run(self, argv: list[str]) -> int:
        """Run the program."""
        args = self._parse_args(argv)
        try:
            graph, schedule, result = self._solve(args.map_file)
        except FlyInError as error:
            print(error, file=self._err)
            return 1
        for line in schedule.lines():
            print(line, file=self._out)
        if not args.quiet:
            self._print_stats(result)
        if not args.no_window:
            self._show_window(graph, result)
        return 0

    def _parse_args(self, argv: list[str]) -> argparse.Namespace:
        """Read the command line."""
        parser = argparse.ArgumentParser(
            prog="fly_in.py",
            description="Route a fleet of drones through a network of zones.")
        parser.add_argument("map_file", help="path of the map file")
        parser.add_argument("--no-window", action="store_true",
                            help="print the output only, open no window")
        parser.add_argument("-q", "--quiet", action="store_true",
                            help="do not print the figures on stderr")
        return parser.parse_args(argv)

    def _solve(self, path: str) -> tuple[Graph, Schedule, SimulationResult]:
        """Parse, plan, and check the plan with the independent referee."""
        graph = MapParser().parse_file(path)
        schedule = Scheduler(graph).build()
        result = Simulator(graph).run(schedule.lines())
        return graph, schedule, result

    def _print_stats(self, result: SimulationResult) -> None:
        """Write the secondary metrics of the subject on stderr."""
        moves = " ".join(str(count) for count in result.moves_per_turn)
        print(f"turns                   : {result.turns}", file=self._err)
        print(f"average turns per drone : {result.average_turns:.2f}",
              file=self._err)
        print(f"total path cost         : {result.total_cost}",
              file=self._err)
        print(f"drones moved per turn   : {moves}", file=self._err)

    def _show_window(self, graph: Graph, result: SimulationResult) -> None:
        """Open the animation; a missing display or pygame is not fatal.

        The renderer is imported here and not at the top of the file, so
        the text output still works on a machine without pygame.
        """
        try:
            from renderer import PygameRenderer
        except ModuleNotFoundError as error:
            if error.name != "pygame":
                raise
            print("warning: pygame is not installed, no window opened "
                  "(run `make install`)", file=self._err)
            return
        try:
            PygameRenderer(graph, result).run(self._max_frames)
        except DisplayError as error:
            print(f"warning: {error}", file=self._err)


def main() -> int:
    """Program entry point: run the app and handle Ctrl+C."""
    try:
        return App().run(sys.argv[1:])
    except KeyboardInterrupt:
        print("Simulation stopped by keyboard.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
