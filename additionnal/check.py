"""Manual check script for the Fly-in project (data layer).

It exercises models.py, graph.py, errors.py, map_parser.py and
pathfinding.py (DistanceMap), plan.py, reservation.py,
space_time_astar.py, scheduler.py and simulator.py.
This is NOT the final entry point (that will be fly_in.py).

Usage:
    python3 check_progress.py            run every check
    python3 check_progress.py MAP.txt    print a summary of one map

Map folders (optional, searched in ./maps):
    maps/**/*.txt            must parse (the provided maps are also
                             compared with their expected counts)
    maps/**/invalid/*.txt    must be REJECTED with an error message
"""

import re
import sys
import tempfile
from functools import partial
from pathlib import Path
from typing import Callable

from errors import (FlyInError, MapFileError, NoPathError, ParseError,
                    SimulationError)
from graph import Graph
from map_parser import MapParser
from models import Connection, Drone, Zone, ZoneType
from pathfinding import DistanceMap
from plan import DronePlan, Leg
from reservation import ReservationTable
from scheduler import Schedule, Scheduler
from simulator import Simulator
from space_time_a_star import SpaceTimeAStar


class CheckRunner:
    """Count and display the results of the checks."""

    GREEN = "\033[32m"
    RED = "\033[31m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RESET = "\033[0m"

    def __init__(self) -> None:
        """Create a runner with no result yet."""
        self.passed: int = 0
        self.failures: list[str] = []
        self._color: bool = sys.stdout.isatty()

    def _paint(self, text: str, code: str) -> str:
        """Wrap text in an ANSI colour (only when writing to a terminal)."""
        if self._color:
            return f"{code}{text}{self.RESET}"
        return text

    def section(self, title: str) -> None:
        """Print a section header."""
        print("\n" + self._paint(f"== {title} ==", self.BOLD))

    def note(self, text: str) -> None:
        """Print an indented information line (not a check)."""
        print("       " + self._paint(text, self.DIM))

    def check(self, label: str, ok: bool, detail: str = "") -> None:
        """Record one result.

        Args:
            label: What is being checked.
            ok: True if the check passed.
            detail: Extra text shown when the check fails.
        """
        if ok:
            self.passed += 1
            print(f"  {self._paint('OK  ', self.GREEN)} {label}")
            return
        self.failures.append(label)
        suffix = f" -> {detail}" if detail else ""
        print(f"  {self._paint('FAIL', self.RED)} {label}{suffix}")

    def expect_error(
        self,
        label: str,
        action: Callable[[], object],
        error_type: type[Exception] = ValueError,
    ) -> None:
        """Check that ``action`` raises ``error_type``.

        Args:
            label: What is being checked.
            action: Function that must fail.
            error_type: Expected exception type.
        """
        try:
            action()
        except error_type:
            self.check(label, True)
        except Exception as exc:
            self.check(label, False, f"{type(exc).__name__}: {exc}")
        else:
            self.check(label, False, "no error was raised")

    def guard(self, label: str, action: Callable[[], None]) -> None:
        """Run a group of checks without letting a crash stop the report.

        Unfinished or buggy code raises exceptions (NotImplementedError,
        TypeError...). Each one becomes ONE failed check and the next
        groups still run.

        Args:
            label: Name of the group.
            action: Function that performs the checks.
        """
        try:
            action()
        except Exception as exc:
            self.check(f"{label}: no crash", False,
                       f"{type(exc).__name__}: {exc}")

    def summary(self) -> int:
        """Print the final report.

        Returns:
            0 if every check passed, 1 otherwise (process exit code).
        """
        total = self.passed + len(self.failures)
        print(f"\n{self.passed}/{total} checks passed")
        if not self.failures:
            return 0
        print("Failed checks:")
        for label in self.failures:
            print(f"  - {label}")
        return 1


class ModelChecks:
    """Checks for models.py."""

    def __init__(self, runner: CheckRunner) -> None:
        """Store the runner."""
        self.runner: CheckRunner = runner

    def run(self) -> None:
        """Run every model check."""
        self.runner.section("models.py")
        self._zone_types()
        self._zones()
        self._connections()
        self._drones()

    def _zone_types(self) -> None:
        """Checks for ZoneType."""
        r = self.runner
        r.check(
            "from_string('restricted')",
            ZoneType.from_string("restricted") is ZoneType.RESTRICTED,
        )
        r.expect_error(
            "from_string('fast') is rejected",
            lambda: ZoneType.from_string("fast"),
        )
        costs = (
            ZoneType.NORMAL.entry_cost,
            ZoneType.PRIORITY.entry_cost,
            ZoneType.RESTRICTED.entry_cost,
        )
        r.check("entry costs normal/priority/restricted = 1/1/2",
                costs == (1, 1, 2), str(costs))
        r.expect_error("blocked has no entry cost",
                       lambda: ZoneType.BLOCKED.entry_cost)
        r.check(
            "is_blocked / is_restricted / is_priority",
            ZoneType.BLOCKED.is_blocked
            and ZoneType.RESTRICTED.is_restricted
            and ZoneType.PRIORITY.is_priority
            and not ZoneType.NORMAL.is_blocked,
        )

    def _zones(self) -> None:
        """Checks for Zone."""
        r = self.runner
        for bad in ("", "a-b", "a b", "a#", "a[", "a]"):
            r.expect_error(
                f"zone name {bad!r} is rejected",
                partial(Zone, bad, 0, 0),
            )
        r.check("negative coordinates are accepted",
                Zone("a", -3, -7).y == -7)
        r.expect_error("max_drones=0 is rejected",
                       lambda: Zone("a", 0, 0, max_drones=0))
        r.expect_error(
            "start and end at the same time is rejected",
            lambda: Zone("a", 0, 0, is_start=True, is_end=True),
        )
        start = Zone("s", 0, 0, max_drones=3, is_start=True)
        end = Zone("g", 1, 0, max_drones=3, is_end=True)
        normal = Zone("n", 2, 0, max_drones=3)
        r.check("start / end capacity is unlimited (None)",
                start.capacity is None and end.capacity is None)
        r.check("normal capacity is max_drones", normal.capacity == 3)
        r.check("can_accept: 2 of 3 yes, 3 of 3 no",
                normal.can_accept(2) and not normal.can_accept(3))
        r.check("start accepts any number of drones",
                start.can_accept(10_000))
        blocked = Zone("b", 0, 0, ZoneType.BLOCKED)
        r.check("blocked shortcut", blocked.is_blocked)
        r.expect_error("blocked zone has no entry cost",
                       lambda: blocked.entry_cost)
        restricted = Zone("r", 0, 0, ZoneType.RESTRICTED)
        r.check("restricted zone costs 2 turns",
                restricted.entry_cost == 2 and restricted.is_restricted)
        r.check("priority shortcut",
                Zone("p", 0, 0, ZoneType.PRIORITY).is_priority)
        first = Zone("a", 0, 0)
        second = Zone("a", 5, 5)
        stranger: object = "a"
        r.check("zones with the same name are equal", first == second)
        r.check("equal zones have equal hashes",
                hash(first) == hash(second))
        r.check("a set keeps one of two same-name zones",
                len({first, second}) == 1)
        r.check("a zone is not equal to a string", first != stranger)

    def _connections(self) -> None:
        """Checks for Connection."""
        r = self.runner
        a = Zone("a", 0, 0)
        b = Zone("b", 1, 0)
        c = Zone("c", 2, 0)
        r.check(
            "make_key is symmetric",
            Connection.make_key("a", "b") == Connection.make_key("b", "a"),
        )
        r.expect_error("connection to itself is rejected",
                       lambda: Connection(a, a))
        r.expect_error("link capacity 0 is rejected",
                       lambda: Connection(a, b, 0))
        r.check("capacity 1 is accepted (it is the default)",
                Connection(a, b).max_link_capacity == 1)
        link = Connection(a, b, 2)
        r.check("same key whatever the order",
                link.key == Connection(b, a).key)
        r.check("connects()",
                link.connects(a) and link.connects(b)
                and not link.connects(c))
        r.check("other_end works both ways",
                link.other_end(a) == b and link.other_end(b) == a)
        r.expect_error("other_end of a stranger is rejected",
                       lambda: link.other_end(c))
        labels = (link.label_from(a), link.label_from(b))
        r.check("label_from works both ways",
                labels == ("a-b", "b-a"), str(labels))
        r.expect_error("label_from a stranger is rejected",
                       lambda: link.label_from(c))

    def _drones(self) -> None:
        """Checks for Drone."""
        r = self.runner
        start = Zone("s", 0, 0, is_start=True)
        r.check("Drone(1).name is 'D1'", Drone(1, start).name == "D1")
        r.check("a drone starts on the given zone",
                Drone(3, start).current_zone == start)
        r.expect_error("drone id 0 is rejected", lambda: Drone(0, start))


class GraphChecks:
    """Checks for graph.py."""

    def __init__(self, runner: CheckRunner) -> None:
        """Store the runner."""
        self.runner: CheckRunner = runner

    def run(self) -> None:
        """Run every graph check."""
        self.runner.section("graph.py")
        self._structure()
        self._reachability()

    def _structure(self) -> None:
        """Checks on registration and queries."""
        r = self.runner
        graph = Graph()
        r.check("empty graph: no start, no end",
                not graph.has_start and not graph.has_end)
        r.expect_error("empty graph: start raises", lambda: graph.start)
        r.expect_error("empty graph: end raises", lambda: graph.end)
        r.expect_error("empty graph: bounds raises", graph.bounds)

        s = Zone("s", 0, -1, is_start=True)
        a = Zone("a", 1, 0)
        b = Zone("b", 2, 3)
        x = Zone("x", 5, 0, ZoneType.BLOCKED)
        g = Zone("g", 3, 0, is_end=True)
        for zone in (s, a, b, x, g):
            graph.add_zone(zone)
        graph.add_connection(Connection(s, a))
        graph.add_connection(Connection(a, g, 2))
        graph.add_connection(Connection(a, x))

        names = [zone.name for zone in graph.zones]
        r.check("zones keep the definition order",
                names == ["s", "a", "b", "x", "g"], str(names))
        r.check("3 connections registered", len(graph.connections) == 3)
        r.check("start and end are registered",
                graph.has_start and graph.has_end
                and graph.start == s and graph.end == g)
        r.check("has_zone / get_zone",
                graph.has_zone("b") and not graph.has_zone("zz")
                and graph.get_zone("b") is b)
        r.expect_error("get_zone of an unknown name raises KeyError",
                       lambda: graph.get_zone("zz"), KeyError)
        around = [zone.name for zone in graph.neighbors(a)]
        r.check("neighbors(a) skips the blocked zone x",
                around == ["s", "g"], str(around))
        r.check("neighbors(b) is empty", graph.neighbors(b) == [])
        r.check("has_connection works in both directions",
                graph.has_connection("a", "g")
                and graph.has_connection("g", "a"))
        r.check("get_connection returns ONE object for a-s and s-a",
                graph.get_connection(a, s) is graph.get_connection(s, a))
        r.check("link capacity is kept",
                graph.get_connection(g, a).max_link_capacity == 2)
        r.expect_error("get_connection of unlinked zones raises KeyError",
                       lambda: graph.get_connection(s, g), KeyError)
        r.check("bounds are (0, -1, 5, 3)",
                graph.bounds() == (0, -1, 5, 3), str(graph.bounds()))

        r.expect_error("reversed duplicate connection is rejected",
                       lambda: graph.add_connection(Connection(a, s)))
        r.expect_error("duplicate zone name is rejected",
                       lambda: graph.add_zone(Zone("a", 9, 9)))
        r.expect_error(
            "second start is rejected",
            lambda: graph.add_zone(Zone("s2", 9, 9, is_start=True)),
        )
        r.expect_error(
            "second end is rejected",
            lambda: graph.add_zone(Zone("g2", 9, 9, is_end=True)),
        )
        r.expect_error(
            "connection to an unknown zone is rejected",
            lambda: graph.add_connection(Connection(a, Zone("ghost", 9, 9))),
        )
        r.check("rejected calls leave the graph unchanged",
                len(graph.zones) == 5 and len(graph.connections) == 3)

        graph.set_nb_drones(5)
        r.check("nb_drones is stored", graph.nb_drones == 5)
        r.expect_error("nb_drones 0 is rejected",
                       lambda: graph.set_nb_drones(0))
        r.check("nb_drones unchanged after a rejected value",
                graph.nb_drones == 5)

    def _chain(self, start_type: ZoneType, middle_type: ZoneType) -> Graph:
        """Build  s - m - g  with the given types for s and m."""
        graph = Graph()
        s = Zone("s", 0, 0, start_type, is_start=True)
        m = Zone("m", 1, 0, middle_type)
        g = Zone("g", 2, 0, is_end=True)
        for zone in (s, m, g):
            graph.add_zone(zone)
        graph.add_connection(Connection(s, m))
        graph.add_connection(Connection(m, g))
        return graph

    def _reachability(self) -> None:
        """Checks on is_goal_reachable (BFS)."""
        r = self.runner
        normal, blocked = ZoneType.NORMAL, ZoneType.BLOCKED
        r.check("s-m-g is reachable",
                self._chain(normal, normal).is_goal_reachable())
        r.check("a blocked middle zone cuts the route",
                not self._chain(normal, blocked).is_goal_reachable())
        r.check("a blocked start cannot reach anything",
                not self._chain(blocked, normal).is_goal_reachable())

        detour = Graph()
        zones = [
            Zone("s", 0, 0, is_start=True), Zone("x", 1, 1, blocked),
            Zone("a", 1, -1), Zone("g", 2, 0, is_end=True),
        ]
        for zone in zones:
            detour.add_zone(zone)
        s, x, a, g = zones
        for one, two in ((s, x), (x, g), (s, a), (a, g)):
            detour.add_connection(Connection(one, two))
        r.check("the BFS goes around a blocked zone",
                detour.is_goal_reachable())

        ring = Graph()
        ring_zones = [
            Zone("s", 0, 0, is_start=True), Zone("c1", 1, 0),
            Zone("c2", 1, 1), Zone("g", 9, 9, is_end=True),
        ]
        for zone in ring_zones:
            ring.add_zone(zone)
        s, c1, c2, _ = ring_zones
        for one, two in ((s, c1), (c1, c2), (c2, s)):
            ring.add_connection(Connection(one, two))
        r.check("a cycle with an isolated goal terminates with False",
                not ring.is_goal_reachable())

        only_end = Graph()
        only_end.add_zone(Zone("g", 0, 0, is_end=True))
        r.expect_error("is_goal_reachable without a start raises",
                       only_end.is_goal_reachable)


class ParserChecks:
    """Checks for map_parser.py (map content given as lines)."""

    BASE: list[str] = [
        "nb_drones: 2",
        "start_hub: s 0 0",
        "end_hub: g 1 0",
        "connection: s-g",
    ]

    def __init__(self, runner: CheckRunner) -> None:
        """Store the runner."""
        self.runner: CheckRunner = runner

    def run(self) -> None:
        """Run every parser check."""
        self.runner.section("map_parser.py - valid content")
        self._valid()
        self.runner.section("map_parser.py - invalid content")
        self._invalid()
        self.runner.section("map_parser.py - files and reuse")
        self._files_and_reuse()

    def _valid(self) -> None:
        """Valid inputs, including the tricky ones."""
        r = self.runner
        base = MapParser().parse_lines(self.BASE)
        r.check("minimal map: 2 zones, 1 connection, 2 drones",
                len(base.zones) == 2 and len(base.connections) == 1
                and base.nb_drones == 2)

        lines = [
            "# a comment first", "", "nb_drones: 3   # trailing comment",
            "start_hub: s 0 -1 [max_drones=9 color=green]",
            "end_hub: g 4 -2 [color=rainbow]",
            "hub: a 1 0 [zone=priority color=lime max_drones=2]",
            "hub: r 2 0 [color=darkred zone=restricted]",
            "hub: b 3 0 [zone=blocked]",
            "  hub :   c    5   5   [ color=red ]  ",
            "connection: s-a [max_link_capacity=2]",
            "connection: a-r",
            "connection: r-g   # another comment",
        ]
        graph = MapParser().parse_lines(lines)
        r.check("comments, blank lines and extra spaces are tolerated",
                graph.nb_drones == 3 and len(graph.zones) == 6
                and len(graph.connections) == 3)
        r.check("negative coordinates",
                graph.start.y == -1 and graph.end.y == -2)
        r.check("max_drones on start is accepted and ignored",
                graph.start.max_drones == 9 and graph.start.capacity is None)
        r.check("any colour word is accepted",
                graph.end.color == "rainbow"
                and graph.get_zone("r").color == "darkred")
        r.check("tags in any order",
                graph.get_zone("r").is_restricted
                and graph.get_zone("a").is_priority
                and graph.get_zone("a").capacity == 2)
        r.check("blocked zone is parsed",
                graph.get_zone("b").is_blocked)
        r.check("max_link_capacity is parsed",
                graph.get_connection(
                    graph.get_zone("s"), graph.get_zone("a")
                ).max_link_capacity == 2)
        r.check("default capacities are 1",
                graph.get_zone("c").max_drones == 1)
        r.check("goal reachable through s-a-r-g",
                graph.is_goal_reachable())

        with_newlines = MapParser().parse_lines(
            [line + "\n" for line in self.BASE])
        r.check("lines ending with \\n (readlines) are accepted",
                len(with_newlines.zones) == 2)
        with_crlf = MapParser().parse_lines(
            [line + "\r\n" for line in self.BASE])
        r.check("Windows line endings are accepted",
                len(with_crlf.zones) == 2)

    def _invalid_cases(self) -> list[tuple[str, list[str], int | None]]:
        """Invalid maps with the line number the error must report."""
        base = self.BASE
        head = base[:3]
        zone_a = ["hub: a 2 2"]
        return [
            ("empty file", [], None),
            ("only comments and blank lines", ["# x", "", "   "], None),
            ("zone before nb_drones", ["start_hub: s 0 0"], 1),
            ("line numbers count comments", ["# c", "", "nb_drones: x"], 3),
            ("nb_drones: 0", ["nb_drones: 0"], 1),
            ("nb_drones: -3", ["nb_drones: -3"], 1),
            ("nb_drones: abc", ["nb_drones: abc"], 1),
            ("nb_drones: 2.5", ["nb_drones: 2.5"], 1),
            ("nb_drones without a value", ["nb_drones:"], 1),
            ("duplicate nb_drones", base[:1] + base, 2),
            ("missing start_hub", ["nb_drones: 2", "end_hub: g 1 0"], None),
            ("missing end_hub", ["nb_drones: 2", "start_hub: s 0 0"], None),
            ("second start_hub", head[:2] + ["start_hub: t 5 5"], 3),
            ("second end_hub", head + ["end_hub: h 5 5"], 4),
            ("duplicate zone name", head + ["hub: s 4 4"], 4),
            ("dash in a zone name", head + ["hub: a-b 2 2"], 4),
            ("float coordinate", head + ["hub: a 1.5 2"], 4),
            ("text coordinate", head + ["hub: a x 2"], 4),
            ("two fields only", head + ["hub: a 2"], 4),
            ("four fields", head + ["hub: a 2 2 2"], 4),
            ("unknown zone type", head + ["hub: a 2 2 [zone=fast]"], 4),
            ("max_drones=0", head + ["hub: a 2 2 [max_drones=0]"], 4),
            ("max_drones=abc", head + ["hub: a 2 2 [max_drones=abc]"], 4),
            ("unclosed bracket", head + ["hub: a 2 2 [color=red"], 4),
            ("stray closing bracket", head + ["hub: a 2 2 color=red]"], 4),
            ("text after the bracket",
             head + ["hub: a 2 2 [color=red] junk"], 4),
            ("two bracket blocks",
             head + ["hub: a 2 2 [color=red] [zone=normal]"], 4),
            ("tag without equal sign", head + ["hub: a 2 2 [color]"], 4),
            ("empty tag value", head + ["hub: a 2 2 [color=]"], 4),
            ("empty tag key", head + ["hub: a 2 2 [=red]"], 4),
            ("unknown tag", head + ["hub: a 2 2 [speed=3]"], 4),
            ("duplicated tag",
             head + ["hub: a 2 2 [color=red color=blue]"], 4),
            ("unknown line prefix", base + ["robot: x"], 5),
            ("line without colon", base + ["hub a 2 2"], 5),
            ("connection to an unknown zone", base + ["connection: s-x"], 5),
            ("connection before the zone definition",
             head + ["connection: s-a"] + zone_a, 4),
            ("duplicate connection, reversed", base + ["connection: g-s"], 5),
            ("connection to itself", base + ["connection: s-s"], 5),
            ("connection without a dash", base + ["connection: s g"], 5),
            ("connection with two dashes", base + ["connection: s-g-x"], 5),
            ("empty zone name in a connection", base + ["connection: -g"], 5),
            ("link capacity 0",
             head + zone_a + ["connection: s-a [max_link_capacity=0]"], 5),
            ("max_drones on a connection",
             head + zone_a + ["connection: s-a [max_drones=2]"], 5),
        ]

    def _invalid(self) -> None:
        """Each invalid map must raise a ParseError on the right line."""
        r = self.runner
        for label, lines, expected_line in self._invalid_cases():
            try:
                MapParser().parse_lines(lines)
            except ParseError as error:
                shown = f"line {expected_line}" in str(error)
                ok = error.line_number == expected_line and (
                    expected_line is None or shown)
                r.check(label, ok,
                        f"got line {error.line_number}, "
                        f"expected {expected_line}: {error}")
            except Exception as exc:
                r.check(label, False, f"CRASH {type(exc).__name__}: {exc}")
            else:
                r.check(label, False, "the map was ACCEPTED")

    def _files_and_reuse(self) -> None:
        """File errors and parser reuse."""
        r = self.runner
        parser = MapParser()
        r.expect_error("missing file raises MapFileError",
                       lambda: parser.parse_file("no/such/file.txt"),
                       MapFileError)
        r.expect_error("a directory raises MapFileError",
                       lambda: parser.parse_file("."), MapFileError)
        with tempfile.TemporaryDirectory() as folder:
            bad = Path(folder) / "bad_encoding.txt"
            bad.write_bytes(b"nb_drones: 2\n\xff\xfe\x00 not utf-8\n")
            r.expect_error("invalid encoding raises MapFileError",
                           lambda: parser.parse_file(str(bad)), MapFileError)
            good = Path(folder) / "good.txt"
            good.write_text("\n".join(self.BASE) + "\n", encoding="utf-8")
            graph = parser.parse_file(str(good))
            r.check("parse_file reads a real file",
                    len(graph.zones) == 2 and graph.nb_drones == 2)

        reused = MapParser()
        try:
            reused.parse_lines(["nb_drones: 2", "hub: broken"])
        except FlyInError:
            pass
        graph = reused.parse_lines(self.BASE)
        r.check("a parser is reusable after an invalid map",
                len(graph.zones) == 2 and len(graph.connections) == 1)
        first = reused.parse_lines(self.BASE)
        second = reused.parse_lines(self.BASE)
        r.check("two parsings give independent graphs", first is not second)


class PathfindingChecks:
    """Checks for pathfinding.py (DistanceMap)."""

    START_DISTANCES: dict[str, int] = {
        "01_linear_path.txt": 3,
        "02_simple_fork.txt": 3,
        "03_basic_capacity.txt": 3,
        "01_dead_end_trap.txt": 4,
        "02_circular_loop.txt": 5,
        "03_priority_puzzle.txt": 4,
        "01_maze_nightmare.txt": 6,
        "02_capacity_hell.txt": 5,
        "03_ultimate_challenge.txt": 12,
        "01_the_impossible_dream.txt": 19,
    }

    PRIORITY_PUZZLE: list[str] = [
        "start_hub: start 0 0 [color=green]",
        "hub: slow_path1 1 -1 [zone=restricted color=red]",
        "hub: slow_path2 2 -1 [color=red]",
        "hub: fast_junction 1 0 [zone=priority color=blue max_drones=2]",
        "hub: fast_path 2 0 [zone=priority color=blue]",
        "hub: merge_point 3 0 [color=yellow max_drones=3]",
        "end_hub: goal 4 0 [color=green]",
        "connection: start-slow_path1",
        "connection: start-fast_junction",
        "connection: slow_path1-slow_path2",
        "connection: slow_path2-merge_point",
        "connection: fast_junction-fast_path",
        "connection: fast_path-merge_point",
        "connection: merge_point-goal [max_link_capacity=2]",
    ]

    def __init__(self, runner: CheckRunner, root: Path) -> None:
        """Store the runner and the folder of the maps."""
        self.runner: CheckRunner = runner
        self.root: Path = root

    def run(self) -> None:
        """Run every DistanceMap check."""
        self.runner.section("pathfinding.py - DistanceMap")
        self.runner.guard("small graphs", self._small_graphs)
        self.runner.guard("priority_puzzle table", self._priority_puzzle)
        self.runner.guard("edge cases", self._edge_cases)
        self.runner.guard("provided maps", self._map_files)

    def _map(self, lines: list[str]) -> Graph:
        """Parse a small map given as lines."""
        return MapParser().parse_lines(["nb_drones: 1"] + lines)

    def _distances(self, graph: Graph) -> list[int | None]:
        """Distances of every zone, in definition order."""
        distance_map = DistanceMap(graph)
        return [distance_map.distance(zone) for zone in graph.zones]

    def _small_graphs(self) -> None:
        """Hand-made graphs whose distances are easy to compute."""
        r = self.runner
        chain = self._map([
            "start_hub: s 0 0", "hub: a 1 0", "end_hub: g 2 0",
            "connection: s-a", "connection: a-g",
        ])
        found = self._distances(chain)
        r.check("chain s-a-g: distances 2, 1, 0", found == [2, 1, 0],
                str(found))

        via_restricted = self._map([
            "start_hub: s 0 0", "hub: r 1 0 [zone=restricted]",
            "end_hub: g 2 0", "connection: s-r", "connection: r-g",
        ])
        found = self._distances(via_restricted)
        r.check("entering a restricted zone costs 2: s=3, r=1, g=0",
                found == [3, 1, 0], str(found))

        restricted_goal = self._map([
            "start_hub: s 0 0", "hub: a 1 0",
            "end_hub: g 2 0 [zone=restricted]",
            "connection: s-a", "connection: a-g",
        ])
        found = self._distances(restricted_goal)
        r.check("a restricted GOAL costs 2 to enter: s=3, a=2, g=0",
                found == [3, 2, 0], str(found))

        priority = self._map([
            "start_hub: s 0 0", "hub: p 1 0 [zone=priority]",
            "end_hub: g 2 0", "connection: s-p", "connection: p-g",
        ])
        found = self._distances(priority)
        r.check("priority zones cost 1 like normal ones: 2, 1, 0",
                found == [2, 1, 0], str(found))

        avoid = self._map([
            "start_hub: s 0 0", "hub: r 1 1 [zone=restricted]",
            "hub: a 1 -1", "end_hub: g 2 0",
            "connection: s-r", "connection: r-g",
            "connection: s-a", "connection: a-g",
        ])
        found = self._distances(avoid)
        r.check("the shorter normal route beats the restricted one",
                found[0] == 2, str(found))

    def _priority_puzzle(self) -> None:
        """The full distance table of 03_priority_puzzle."""
        graph = self._map(self.PRIORITY_PUZZLE)
        distance_map = DistanceMap(graph)
        found = {zone.name: distance_map.distance(zone)
                 for zone in graph.zones}
        expected: dict[str, int | None] = {
            "start": 4, "slow_path1": 3, "slow_path2": 2,
            "fast_junction": 3, "fast_path": 2, "merge_point": 1,
            "goal": 0,
        }
        self.runner.check("priority_puzzle: full table",
                          found == expected, str(found))

    def _edge_cases(self) -> None:
        """Blocked zones, isolated zones, cycles, missing goal."""
        r = self.runner
        blocked_goal = self._map([
            "start_hub: s 0 0", "end_hub: g 1 0 [zone=blocked]",
            "connection: s-g",
        ])
        distance_map = DistanceMap(blocked_goal)
        r.check("blocked goal: nothing is reachable",
                distance_map.distance(blocked_goal.start) is None
                and not distance_map.is_reachable(blocked_goal.start))

        wall = self._map([
            "start_hub: s 0 0", "hub: x 1 0 [zone=blocked]",
            "end_hub: g 2 0", "connection: s-x", "connection: x-g",
        ])
        distance_map = DistanceMap(wall)
        r.check("a blocked zone cuts the only route",
                distance_map.distance(wall.start) is None)

        detour = self._map([
            "start_hub: s 0 0", "hub: x 1 1 [zone=blocked]",
            "hub: a 1 -1", "hub: b 2 -1", "end_hub: g 3 0",
            "connection: s-x", "connection: x-g",
            "connection: s-a", "connection: a-b", "connection: b-g",
        ])
        distance_map = DistanceMap(detour)
        r.check("the route goes around a blocked zone (3 turns)",
                distance_map.distance(detour.start) == 3)

        isolated = self._map([
            "start_hub: s 0 0", "hub: lost 5 5", "end_hub: g 1 0",
            "connection: s-g",
        ])
        distance_map = DistanceMap(isolated)
        lost = isolated.get_zone("lost")
        r.check("an isolated zone has distance None",
                distance_map.distance(lost) is None
                and not distance_map.is_reachable(lost))
        r.check("is_reachable is True on a reachable zone",
                distance_map.is_reachable(isolated.start))

        ring = self._map([
            "start_hub: s 0 0", "hub: c1 1 0", "hub: c2 1 1",
            "end_hub: g 2 1", "connection: s-c1", "connection: c1-c2",
            "connection: c2-s", "connection: c2-g",
        ])
        found = self._distances(ring)
        r.check("a cycle terminates: s=2, c1=2, c2=1, g=0",
                found == [2, 2, 1, 0], str(found))

        r.expect_error("a graph without an end zone is rejected",
                       lambda: DistanceMap(Graph()))

    def _problem(self, graph: Graph, distance_map: DistanceMap) -> str:
        """Describe the first broken invariant of a distance map.

        Two properties hold for a correct Dijkstra:
            - no neighbour offers a shorter way:
              d(zone) <= d(neighbour) + cost of entering the neighbour;
            - some neighbour explains the distance exactly.

        Returns:
            A description of the problem, or "" if all is well.
        """
        for zone in graph.zones:
            own = distance_map.distance(zone)
            if own is None:
                continue
            if zone == graph.end:
                if own != 0:
                    return "the goal is not at distance 0"
                continue
            best: int | None = None
            for other in graph.neighbors(zone):
                theirs = distance_map.distance(other)
                if theirs is None:
                    continue
                via = theirs + other.entry_cost
                if own > via:
                    return f"{zone.name}: {own} but {via} via {other.name}"
                if best is None or via < best:
                    best = via
            if best != own:
                return f"{zone.name}: no neighbour explains {own}"
        return ""

    def _map_files(self) -> None:
        """Distances on the map files (and a cross-check with the BFS)."""
        r = self.runner
        if not self.root.is_dir():
            r.note(f"folder {self.root}/ not found: skipped")
            return
        for path in sorted(self.root.rglob("*.txt")):
            relative = path.relative_to(self.root)
            if "invalid" in relative.parts:
                continue
            try:
                graph = MapParser().parse_file(str(path))
            except FlyInError:
                continue
            distance_map = DistanceMap(graph)
            start_distance = distance_map.distance(graph.start)
            problem = self._problem(graph, distance_map)
            r.check(f"{relative}: Dijkstra invariants hold",
                    problem == "", problem)
            r.check(
                f"{relative}: Dijkstra and BFS agree on reachability",
                distance_map.is_reachable(graph.start)
                == graph.is_goal_reachable(),
            )
            expected = self.START_DISTANCES.get(path.name)
            if expected is not None:
                r.check(
                    f"{relative}: distance(start) = {expected}",
                    start_distance == expected,
                    f"got {start_distance}",
                )


class MapBuilder:
    """Helpers to build small graphs and legs for the checks."""

    @staticmethod
    def graph(lines: list[str]) -> Graph:
        """Parse a small map given as lines (4 drones)."""
        return MapParser().parse_lines(["nb_drones: 4"] + lines)

    @staticmethod
    def link(graph: Graph, first: str, second: str) -> Connection:
        """Connection between two zones given by name."""
        return graph.get_connection(
            graph.get_zone(first), graph.get_zone(second))

    @staticmethod
    def leg(graph: Graph, origin: str, destination: str, depart: int) -> Leg:
        """A leg from ``origin`` to ``destination`` leaving at ``depart``."""
        return Leg(graph.get_zone(origin),
                   MapBuilder.link(graph, origin, destination), depart)


class PlanChecks:
    """Checks for plan.py (Leg and DronePlan)."""

    LINES: list[str] = [
        "start_hub: s 0 0", "hub: a 1 0", "hub: r 1 1 [zone=restricted]",
        "hub: x 3 3 [zone=blocked]", "end_hub: g 2 0",
        "connection: s-a", "connection: s-r", "connection: a-g",
        "connection: r-g", "connection: a-x",
    ]

    def __init__(self, runner: CheckRunner) -> None:
        """Store the runner."""
        self.runner: CheckRunner = runner

    def run(self) -> None:
        """Run every plan check."""
        self.runner.section("plan.py - Leg and DronePlan")
        self.runner.guard("legs", self._legs)
        self.runner.guard("plans", self._plans)
        self.runner.guard("invalid plans", self._invalid_plans)

    def _names(self, plan: DronePlan, turns: range) -> list[str | None]:
        """Zone name at the end of each turn (None while in flight)."""
        found: list[str | None] = []
        for turn in turns:
            zone = plan.position_at(turn)
            found.append(None if zone is None else zone.name)
        return found

    def _legs(self) -> None:
        """Checks for Leg."""
        r = self.runner
        graph = MapBuilder.graph(self.LINES)
        s, a, g = (graph.get_zone(n) for n in ("s", "a", "g"))
        walk = MapBuilder.leg(graph, "s", "a", 1)
        r.check("normal leg: to a, duration 1, arrives turn 1",
                walk.destination == a and walk.duration == 1
                and walk.arrive_turn == 1 and not walk.is_flight)
        shown = (walk.output_at(0), walk.output_at(1), walk.output_at(2))
        r.check("normal leg prints the zone at its own turn only",
                shown == (None, "a", None), str(shown))
        flight = MapBuilder.leg(graph, "s", "r", 1)
        r.check("flight leg: duration 2, arrives turn 2, is a flight",
                flight.duration == 2 and flight.arrive_turn == 2
                and flight.is_flight)
        steps = [flight.output_at(t) for t in (1, 2, 3)]
        r.check("flight leg prints the connection, then the zone",
                steps == ["s-r", "r", None], str(steps))
        back = MapBuilder.leg(graph, "a", "s", 5)
        r.check("a leg can start from either end of the connection",
                back.destination == s and back.arrive_turn == 5)
        link_sa = MapBuilder.link(graph, "s", "a")
        link_ax = MapBuilder.link(graph, "a", "x")
        r.expect_error("depart turn 0 is rejected",
                       lambda: Leg(s, link_sa, 0))
        r.expect_error("an origin outside the connection is rejected",
                       lambda: Leg(g, link_sa, 1))
        r.expect_error("a move into a blocked zone is rejected",
                       lambda: Leg(a, link_ax, 1))
        r.check("repr does not crash and is not empty", repr(walk) != "")

    def _plans(self) -> None:
        """Checks for valid DronePlan objects."""
        r = self.runner
        graph = MapBuilder.graph(self.LINES)
        leg = MapBuilder.leg
        waiting = DronePlan(
            3, [leg(graph, "s", "a", 1), leg(graph, "a", "g", 4)])
        names = self._names(waiting, range(6))
        r.check("position_at 0..5 of  s->a (turn 1), a->g (turn 4)",
                names == ["s", "a", "a", "a", "g", "g"], str(names))
        r.check("name is D3 and arrival_turn is 4",
                waiting.name == "D3" and waiting.arrival_turn == 4)
        tokens = [waiting.output_at(t) for t in range(1, 6)]
        r.check("waiting turns print nothing",
                tokens == ["D3-a", None, None, "D3-g", None], str(tokens))
        flying = DronePlan(
            1, [leg(graph, "s", "r", 1), leg(graph, "r", "g", 3)])
        names = self._names(flying, range(4))
        r.check("flight: s, in flight (None), r, g",
                names == ["s", None, "r", "g"], str(names))
        tokens = [flying.output_at(t) for t in (1, 2, 3)]
        r.check("flight prints D1-s-r, then D1-r, then D1-g",
                tokens == ["D1-s-r", "D1-r", "D1-g"], str(tokens))
        r.check("flight plan arrives at turn 3", flying.arrival_turn == 3)
        source = [leg(graph, "s", "a", 1), leg(graph, "a", "g", 2)]
        plan = DronePlan(1, source)
        source.clear()
        r.check("the plan keeps its own copy of the legs",
                len(plan.legs) == 2)
        r.check("repr does not crash and is not empty", repr(plan) != "")

    def _invalid_plans(self) -> None:
        """DronePlan must reject impossible schedules."""
        r = self.runner
        graph = MapBuilder.graph(self.LINES)
        leg = MapBuilder.leg
        first = leg(graph, "s", "a", 1)
        r.expect_error("drone id 0 is rejected",
                       lambda: DronePlan(0, [first]))
        r.expect_error("a plan without legs is rejected",
                       lambda: DronePlan(1, []))
        r.expect_error(
            "legs that are not chained are rejected",
            lambda: DronePlan(1, [first, leg(graph, "s", "r", 3)]))
        r.expect_error(
            "leaving before arriving is rejected",
            lambda: DronePlan(1, [first, leg(graph, "a", "g", 1)]))
        r.expect_error(
            "leaving on the arrival turn of a flight is rejected",
            lambda: DronePlan(1, [leg(graph, "s", "r", 1),
                                  leg(graph, "r", "g", 2)]))
        later = DronePlan(
            1, [leg(graph, "s", "r", 1), leg(graph, "r", "g", 3)])
        r.check("leaving the turn AFTER the arrival is accepted",
                later.arrival_turn == 3)
        r.expect_error("position_at(-1) is rejected",
                       lambda: later.position_at(-1))


class ReservationChecks:
    """Checks for reservation.py (ReservationTable)."""

    def __init__(self, runner: CheckRunner) -> None:
        """Store the runner."""
        self.runner: CheckRunner = runner

    def run(self) -> None:
        """Run every reservation check."""
        self.runner.section("reservation.py - ReservationTable")
        self.runner.guard("train", self._train)
        self.runner.guard("waiting", self._waiting)
        self.runner.guard("zone capacity", self._zone_capacity)
        self.runner.guard("swap", self._swap)
        self.runner.guard("flight", self._flight)
        self.runner.guard("arrival capacity", self._arrival_capacity)
        self.runner.guard("unlimited zones", self._unlimited)
        self.runner.guard("errors", self._errors)

    def _train(self) -> None:
        """A line of zones: a drone leaves a zone, another enters it."""
        r = self.runner
        graph = MapBuilder.graph([
            "start_hub: s 0 0", "hub: a 1 0", "hub: b 2 0",
            "end_hub: g 3 0", "connection: s-a", "connection: a-b",
            "connection: b-g",
        ])
        leg, link = MapBuilder.leg, MapBuilder.link
        s, a, b, g = (graph.get_zone(n) for n in ("s", "a", "b", "g"))
        table = ReservationTable(graph)
        table.reserve_plan(DronePlan(1, [
            leg(graph, "s", "a", 1), leg(graph, "a", "b", 2),
            leg(graph, "b", "g", 3)]))
        loads = (table.zone_load(a, 1), table.zone_load(b, 2),
                 table.zone_load(g, 3))
        r.check("D1 is in a at turn 1, in b at turn 2, in g at turn 3",
                loads == (1, 1, 1), str(loads))
        r.check("a drone that left a zone no longer counts in it",
                table.zone_load(a, 2) == 0 and table.zone_load(b, 3) == 0)
        links = (table.link_load(link(graph, "s", "a"), 1),
                 table.link_load(link(graph, "a", "b"), 2),
                 table.link_load(link(graph, "b", "g"), 3))
        r.check("each normal move reserves its connection at its turn",
                links == (1, 1, 1), str(links))
        r.check("the connection is free at the other turns",
                table.link_load(link(graph, "s", "a"), 2) == 0)
        r.check("can_stay: a is full at turn 1, free at turn 2",
                not table.can_stay(a, 1) and table.can_stay(a, 2))
        r.check("can_move s->a at turn 1 is refused (a is taken)",
                not table.can_move(s, a, 1))
        r.check("can_move s->a at turn 2 is accepted (D1 left a)",
                table.can_move(s, a, 2))
        table.reserve_plan(DronePlan(2, [
            leg(graph, "s", "a", 2), leg(graph, "a", "b", 3),
            leg(graph, "b", "g", 4)]))
        r.check("D2 follows D1 one turn behind: a holds ONE drone each turn",
                (table.zone_load(a, 1), table.zone_load(a, 2),
                 table.zone_load(a, 3)) == (1, 1, 0))

    def _waiting(self) -> None:
        """Waiting drones occupy their zone and must be recorded."""
        r = self.runner
        graph = MapBuilder.graph([
            "start_hub: s 0 0", "hub: a 1 0", "end_hub: g 2 0",
            "connection: s-a", "connection: a-g",
        ])
        leg = MapBuilder.leg
        s, a, g = (graph.get_zone(n) for n in ("s", "a", "g"))
        table = ReservationTable(graph)
        table.reserve_plan(DronePlan(1, [
            leg(graph, "s", "a", 3), leg(graph, "a", "g", 6)]))
        at_start = (table.zone_load(s, 1), table.zone_load(s, 2),
                    table.zone_load(s, 3))
        r.check("waiting at the start is recorded (turns 1 and 2)",
                at_start == (1, 1, 0), str(at_start))
        in_a = tuple(table.zone_load(a, t) for t in (3, 4, 5, 6))
        r.check("arrival and waiting in a are recorded (turns 3, 4, 5)",
                in_a == (1, 1, 1, 0), str(in_a))
        r.check("nothing is recorded after the last arrival",
                table.zone_load(g, 6) == 1 and table.zone_load(g, 7) == 0)
        r.check("a waiting drone blocks its zone: a is full at turn 4",
                not table.can_stay(a, 4) and not table.can_move(s, a, 4))

    def _zone_capacity(self) -> None:
        """A zone with max_drones=2."""
        r = self.runner
        graph = MapBuilder.graph([
            "start_hub: s 0 0", "hub: z 1 0 [max_drones=2]",
            "end_hub: g 2 0", "connection: s-z", "connection: z-g",
        ])
        z = graph.get_zone("z")
        table = ReservationTable(graph)
        table.reserve_stay(z, 3)
        r.check("capacity 2: a second drone is accepted",
                table.can_stay(z, 3))
        table.reserve_stay(z, 3)
        r.check("capacity 2: a third drone is refused",
                not table.can_stay(z, 3))
        r.check("another turn is not affected", table.can_stay(z, 4))

    def _swap(self) -> None:
        """Two drones exchanging places use the connection together."""
        r = self.runner
        for capacity, accepted in ((1, False), (2, True)):
            graph = MapBuilder.graph([
                "start_hub: s 0 0", "hub: x 1 0", "hub: y 2 0",
                "end_hub: g 3 0", "connection: s-x",
                f"connection: x-y [max_link_capacity={capacity}]",
                "connection: y-g",
            ])
            x, y = graph.get_zone("x"), graph.get_zone("y")
            table = ReservationTable(graph)
            table.reserve_leg(MapBuilder.leg(graph, "x", "y", 5))
            word = "accepted" if accepted else "refused"
            r.check(f"swap x<->y, link capacity {capacity}: {word}",
                    table.can_move(y, x, 5) == accepted)
            r.check("x-y and y-x share ONE counter",
                    table.link_load(MapBuilder.link(graph, "y", "x"), 5)
                    == 1)

    def _flight(self) -> None:
        """A flight toward a restricted zone keeps the link 2 turns."""
        r = self.runner
        graph = MapBuilder.graph([
            "start_hub: s 0 0", "hub: r 1 0 [zone=restricted]",
            "end_hub: g 2 0", "connection: s-r", "connection: r-g",
        ])
        s, zone_r = graph.get_zone("s"), graph.get_zone("r")
        link = MapBuilder.link(graph, "s", "r")
        table = ReservationTable(graph)
        table.reserve_plan(DronePlan(1, [
            MapBuilder.leg(graph, "s", "r", 1),
            MapBuilder.leg(graph, "r", "g", 3)]))
        busy = tuple(table.link_load(link, t) for t in (1, 2, 3))
        r.check("a flight occupies the connection at turns 1 and 2",
                busy == (1, 1, 0), str(busy))
        inside = tuple(table.zone_load(zone_r, t) for t in (1, 2, 3))
        r.check("in flight the drone is in NO zone; it arrives at turn 2",
                inside == (0, 1, 0), str(inside))
        r.check("a second flight at turn 1 is refused (link busy)",
                not table.can_move(s, zone_r, 1))
        r.check("a second flight at turn 2 is refused (link still busy)",
                not table.can_move(s, zone_r, 2))
        r.check("a second flight at turn 3 is accepted",
                table.can_move(s, zone_r, 3))

    def _arrival_capacity(self) -> None:
        """The destination must have room at the ARRIVAL turn."""
        r = self.runner
        graph = MapBuilder.graph([
            "start_hub: s 0 0", "hub: t 0 1",
            "hub: r 1 0 [zone=restricted]", "end_hub: g 2 0",
            "connection: s-t", "connection: s-r", "connection: t-r",
            "connection: r-g",
        ])
        t, zone_r = graph.get_zone("t"), graph.get_zone("r")
        table = ReservationTable(graph)
        table.reserve_plan(DronePlan(1, [
            MapBuilder.leg(graph, "s", "r", 1),
            MapBuilder.leg(graph, "r", "g", 5)]))
        held = tuple(table.zone_load(zone_r, k) for k in (2, 3, 4, 5))
        r.check("D1 sits in r during turns 2, 3 and 4",
                held == (1, 1, 1, 0), str(held))
        answers = (table.can_move(t, zone_r, 2),
                   table.can_move(t, zone_r, 3),
                   table.can_move(t, zone_r, 4))
        r.check("flights arriving at turns 3 and 4 are refused, "
                "the one arriving at turn 5 is accepted",
                answers == (False, False, True), str(answers))

    def _unlimited(self) -> None:
        """Start and end have no limit, but connections still do."""
        r = self.runner
        graph = MapBuilder.graph([
            "start_hub: s 0 0", "end_hub: g 1 0",
            "connection: s-g [max_link_capacity=5]",
        ])
        s, g = graph.get_zone("s"), graph.get_zone("g")
        table = ReservationTable(graph)
        move = MapBuilder.leg(graph, "s", "g", 1)
        for _ in range(4):
            table.reserve_leg(move)
        r.check("the end zone accepts any number of drones",
                table.can_move(s, g, 1))
        table.reserve_leg(move)
        r.check("but the connection capacity (5) still applies",
                not table.can_move(s, g, 1))
        for _ in range(10):
            table.reserve_stay(s, 3)
        r.check("the start zone accepts any number of waiting drones",
                table.can_stay(s, 3))

    def _errors(self) -> None:
        """Misuse must be reported."""
        r = self.runner
        graph = MapBuilder.graph([
            "start_hub: s 0 0", "hub: a 1 0", "hub: x 2 0 [zone=blocked]",
            "end_hub: g 3 0", "connection: s-a", "connection: a-x",
            "connection: x-g",
        ])
        s, a, x, g = (graph.get_zone(n) for n in ("s", "a", "x", "g"))
        table = ReservationTable(graph)
        r.expect_error("can_move between zones that are not linked",
                       lambda: table.can_move(s, g, 1), KeyError)
        r.expect_error("can_move into a blocked zone",
                       lambda: table.can_move(a, x, 1))


class PlanAuditor:
    """Replay plans on a fresh reservation table (shared by the checks)."""

    @staticmethod
    def replay(graph: Graph, plans: list[DronePlan]) -> str:
        """Replay the plans on a FRESH table and ask it for permission.

        Each wait must pass can_stay and each leg must pass can_move,
        before the plan is recorded. This only checks that the planner
        and the table agree; the independent simulator will
        check the rules themselves.

        Args:
            graph: The map.
            plans: The plans to replay, in planning order.

        Returns:
            A description of the first refusal, or "".
        """
        table = ReservationTable(graph)
        for plan in plans:
            previous = 0
            for leg in plan.legs:
                for turn in range(previous + 1, leg.depart_turn):
                    if not table.can_stay(leg.origin, turn):
                        return (f"{plan.name} waits in {leg.origin.name}"
                                f" at turn {turn}: refused")
                if not table.can_move(
                        leg.origin, leg.destination, leg.depart_turn):
                    return f"{plan.name} {leg!r}: refused"
                previous = leg.arrive_turn
            table.reserve_plan(plan)
        return ""


class SpaceTimeChecks:
    """Checks for space_time_astar.py (SpaceTimeAStar)."""

    TARGETS: dict[str, int] = {
        "01_linear_path.txt": 6,
        "02_simple_fork.txt": 8,
        "03_basic_capacity.txt": 6,
        "01_dead_end_trap.txt": 12,
        "02_circular_loop.txt": 15,
        "03_priority_puzzle.txt": 12,
        "01_maze_nightmare.txt": 30,
        "02_capacity_hell.txt": 35,
        "03_ultimate_challenge.txt": 45,
    }

    OPTIMUM: dict[str, int] = {
        "01_linear_path.txt": 4,
        "02_simple_fork.txt": 4,
        "03_basic_capacity.txt": 4,
        "01_dead_end_trap.txt": 8,
    }
    RECORD_NAME: str = "01_the_impossible_dream.txt"
    RECORD: int = 45

    def __init__(self, runner: CheckRunner, root: Path) -> None:
        """Store the runner and the folder of the maps."""
        self.runner: CheckRunner = runner
        self.root: Path = root

    def run(self) -> None:
        """Run every space-time A* check."""
        self.runner.section("space_time_astar.py - SpaceTimeAStar")
        self.runner.guard("a line", self._line)
        self.runner.guard("a flight", self._flight)
        self.runner.guard("priority tie-break", self._priority)
        self.runner.guard("errors", self._errors)
        self.runner.guard("provided maps", self._map_files)

    def _tools(
        self, lines: list[str]
    ) -> tuple[Graph, ReservationTable, SpaceTimeAStar]:
        """Build a small map with its table and its planner."""
        graph = MapBuilder.graph(lines)
        table = ReservationTable(graph)
        planner = SpaceTimeAStar(graph, DistanceMap(graph), table)
        return graph, table, planner

    def _departs(self, plan: DronePlan) -> list[int]:
        """Departure turns of the legs of a plan."""
        return [leg.depart_turn for leg in plan.legs]

    def _line(self) -> None:
        """Two drones in a line: the second one waits, then follows."""
        r = self.runner
        _, table, planner = self._tools([
            "start_hub: s 0 0", "hub: a 1 0", "hub: b 2 0",
            "end_hub: g 3 0", "connection: s-a", "connection: a-b",
            "connection: b-g",
        ])
        first = planner.find_plan(1, 10)
        r.check("D1 alone: legs depart at turns 1, 2, 3, arrival 3",
                self._departs(first) == [1, 2, 3]
                and first.arrival_turn == 3, str(self._departs(first)))
        r.check("the plan is made for the requested drone",
                first.drone_id == 1 and first.name == "D1")
        r.check("max_turn_for(k) is k times distance(start): 3, 12",
                planner.max_turn_for(1) == 3
                and planner.max_turn_for(4) == 12)
        r.expect_error("max_turn_for(0) is rejected",
                       lambda: planner.max_turn_for(0))
        table.reserve_plan(first)
        second = planner.find_plan(2, 10)
        r.check("D2 waits one turn then follows: departs 2, 3, 4",
                self._departs(second) == [2, 3, 4]
                and second.arrival_turn == 4, str(self._departs(second)))
        r.check("a plan ends on the end zone",
                second.legs[-1].destination.is_end
                and second.legs[0].origin.is_start)

    def _flight(self) -> None:
        """A flight keeps the link busy for two turns."""
        r = self.runner
        _, table, planner = self._tools([
            "start_hub: s 0 0", "hub: r 1 0 [zone=restricted]",
            "end_hub: g 2 0", "connection: s-r", "connection: r-g",
        ])
        first = planner.find_plan(1, 10)
        r.check("D1 alone: departs 1 and 3, arrival 3",
                self._departs(first) == [1, 3]
                and first.arrival_turn == 3, str(self._departs(first)))
        table.reserve_plan(first)
        second = planner.find_plan(2, 10)
        r.check("D2: the link is busy at turns 1 and 2, departs 3 and 5",
                self._departs(second) == [3, 5]
                and second.arrival_turn == 5, str(self._departs(second)))

    def _priority(self) -> None:
        """Equal routes: the priority zone is preferred."""
        r = self.runner
        _, _, planner = self._tools([
            "start_hub: s 0 0", "hub: n 1 0", "hub: p 1 1 [zone=priority]",
            "end_hub: g 2 0", "connection: s-n", "connection: n-g",
            "connection: s-p", "connection: p-g",
        ])
        plan = planner.find_plan(1, 10)
        r.check("same duration: the route through the priority zone",
                plan.legs[0].destination.name == "p"
                and plan.arrival_turn == 2,
                plan.legs[0].destination.name)

    def _errors(self) -> None:
        """No route, or not within the horizon."""
        r = self.runner
        _, _, planner = self._tools([
            "start_hub: s 0 0", "hub: x 1 0 [zone=blocked]",
            "end_hub: g 2 0", "connection: s-x", "connection: x-g",
        ])
        r.expect_error("a blocked wall raises NoPathError",
                       lambda: planner.find_plan(1, 10), NoPathError)
        r.expect_error("max_turn_for on a blocked wall raises NoPathError",
                       lambda: planner.max_turn_for(1), NoPathError)
        _, _, line = self._tools([
            "start_hub: s 0 0", "hub: a 1 0", "hub: b 2 0",
            "end_hub: g 3 0", "connection: s-a", "connection: a-b",
            "connection: b-g",
        ])
        r.expect_error("max_turn 2 on a 3-turn line raises NoPathError",
                       lambda: line.find_plan(1, 2), NoPathError)
        r.check("max_turn 3 on the same line works",
                line.find_plan(1, 3).arrival_turn == 3)

    def _schedule(self, graph: Graph) -> list[DronePlan]:
        """Naive scheduling: the drones are planned one after the other.

        Drone k gets max_turn_for(k) = k * distance(start), a bound that
        always works (see SpaceTimeAStar.max_turn_for).
        """
        table = ReservationTable(graph)
        planner = SpaceTimeAStar(graph, DistanceMap(graph), table)
        plans: list[DronePlan] = []
        for number in range(1, graph.nb_drones + 1):
            plan = planner.find_plan(number, planner.max_turn_for(number))
            table.reserve_plan(plan)
            plans.append(plan)
        return plans

    def _map_files(self) -> None:
        """Naive scheduling on the provided maps."""
        r = self.runner
        if not self.root.is_dir():
            r.note(f"folder {self.root}/ not found: skipped")
            return
        for path in sorted(self.root.rglob("*.txt")):
            known = path.name in PathfindingChecks.START_DISTANCES
            if "invalid" in path.relative_to(self.root).parts or not known:
                continue
            label = str(path.relative_to(self.root))
            try:
                graph = MapParser().parse_file(str(path))
            except FlyInError:
                continue
            plans = self._schedule(graph)
            makespan = max(plan.arrival_turn for plan in plans)
            expected = PathfindingChecks.START_DISTANCES[path.name]
            r.check(f"{label}: the first drone arrives at turn {expected}",
                    plans[0].arrival_turn == expected,
                    f"got {plans[0].arrival_turn}")
            problem = PlanAuditor.replay(graph, plans)
            r.check(f"{label}: the table accepts every plan",
                    problem == "", problem)
            target = self.TARGETS.get(path.name)
            if target is not None:
                r.check(f"{label}: makespan {makespan} <= target {target}",
                        makespan <= target)
            optimum = self.OPTIMUM.get(path.name)
            if optimum is not None:
                r.check(f"{label}: optimal makespan {optimum}",
                        makespan == optimum, f"got {makespan}")
            if path.name == self.RECORD_NAME:
                r.note(f"{label}: makespan {makespan} "
                       f"(record to beat: {self.RECORD})")


class SchedulerChecks:
    """Checks for scheduler.py (Schedule and Scheduler)."""

    LINE: list[str] = [
        "start_hub: s 0 0", "hub: a 1 0", "hub: b 2 0", "end_hub: g 3 0",
        "connection: s-a", "connection: a-b", "connection: b-g",
    ]
    SHORTCUT: list[str] = [
        "start_hub: s 0 0", "hub: a 1 0", "end_hub: g 2 0",
        "connection: s-a", "connection: a-g", "connection: s-g",
    ]

    def __init__(self, runner: CheckRunner, root: Path) -> None:
        """Store the runner and the folder of the maps."""
        self.runner: CheckRunner = runner
        self.root: Path = root

    def run(self) -> None:
        """Run every scheduler check."""
        self.runner.section("scheduler.py - Schedule and Scheduler")
        self.runner.guard("Schedule", self._schedule_object)
        self.runner.guard("output lines", self._lines)
        self.runner.guard("is_better_than", self._comparison)
        self.runner.guard("Scheduler", self._scheduler)
        self.runner.guard("provided maps", self._map_files)

    def _line_plans(self) -> tuple[DronePlan, DronePlan]:
        """Two drones in a line s-a-b-g, the second one turn behind."""
        graph = MapBuilder.graph(self.LINE)
        leg = MapBuilder.leg
        first = DronePlan(1, [leg(graph, "s", "a", 1),
                              leg(graph, "a", "b", 2),
                              leg(graph, "b", "g", 3)])
        second = DronePlan(2, [leg(graph, "s", "a", 2),
                               leg(graph, "a", "b", 3),
                               leg(graph, "b", "g", 4)])
        return first, second

    def _schedule_object(self) -> None:
        """Construction, sorting, makespan and cost."""
        r = self.runner
        first, second = self._line_plans()
        schedule = Schedule([second, first])
        ids = [plan.drone_id for plan in schedule.plans]
        r.check("plans are sorted by drone id", ids == [1, 2], str(ids))
        r.check("makespan is the latest arrival (4)",
                schedule.makespan == 4)
        r.check("total_cost sums the leg durations (6)",
                schedule.total_cost == 6, str(schedule.total_cost))
        r.expect_error("an empty schedule is rejected",
                       lambda: Schedule([]))
        r.expect_error("two plans with the same id are rejected",
                       lambda: Schedule([first, first]))

    def _lines(self) -> None:
        """The output of the subject, turn by turn."""
        r = self.runner
        first, second = self._line_plans()
        schedule = Schedule([second, first])
        expected = ["D1-a", "D1-b D2-a", "D1-g D2-b", "D2-g"]
        r.check("2 drones in a line: the 4 output lines",
                schedule.lines() == expected, str(schedule.lines()))
        r.check("turn_line(2) is 'D1-b D2-a'",
                schedule.turn_line(2) == "D1-b D2-a")
        r.check("one line per turn: len(lines) == makespan",
                len(schedule.lines()) == schedule.makespan)

        graph = MapBuilder.graph([
            "start_hub: s 0 0", "hub: r 1 0 [zone=restricted]",
            "end_hub: g 2 0", "connection: s-r", "connection: r-g",
        ])
        flight = Schedule([DronePlan(1, [
            MapBuilder.leg(graph, "s", "r", 1),
            MapBuilder.leg(graph, "r", "g", 3)])])
        r.check("a flight prints the connection, then the zone",
                flight.lines() == ["D1-s-r", "D1-r", "D1-g"],
                str(flight.lines()))
        r.check("total_cost counts 2 for a flight and 1 for a move",
                flight.total_cost == 3)

        direct = MapBuilder.graph([
            "start_hub: s 0 0", "end_hub: g 1 0", "connection: s-g",
        ])
        late = Schedule([DronePlan(1, [
            MapBuilder.leg(direct, "s", "g", 3)])])
        r.check("a turn where nobody moves is an EMPTY line, not skipped",
                late.lines() == ["", "", "D1-g"], str(late.lines()))

    def _comparison(self) -> None:
        """Fewer turns wins; on equal turns, the lower cost wins."""
        r = self.runner
        graph = MapBuilder.graph(self.SHORTCUT)
        leg = MapBuilder.leg
        fast = Schedule([DronePlan(1, [leg(graph, "s", "g", 1)])])
        slow = Schedule([DronePlan(1, [leg(graph, "s", "a", 1),
                                       leg(graph, "a", "g", 2)])])
        cheap = Schedule([DronePlan(1, [leg(graph, "s", "g", 2)])])
        r.check("fewer turns is better",
                fast.is_better_than(slow) and not slow.is_better_than(fast))
        r.check("same turns: the lower total cost is better",
                cheap.is_better_than(slow)
                and not slow.is_better_than(cheap))
        r.check("a schedule is not better than itself",
                not fast.is_better_than(fast))

    def _scheduler(self) -> None:
        """Scheduler on hand-made maps."""
        r = self.runner
        wall = MapParser().parse_lines([
            "nb_drones: 2", "start_hub: s 0 0", "hub: x 1 0 [zone=blocked]",
            "end_hub: g 2 0", "connection: s-x", "connection: x-g",
        ])
        r.expect_error("a goal that cannot be reached raises NoPathError",
                       lambda: Scheduler(wall), NoPathError)
        line = MapParser().parse_lines(["nb_drones: 2"] + self.LINE)
        scheduler = Scheduler(line)
        built = scheduler.build()
        r.check("2 drones in a line: the 4 output lines",
                built.lines() == ["D1-a", "D1-b D2-a", "D1-g D2-b", "D2-g"],
                str(built.lines()))
        r.check("build() is never worse than plan_greedy()",
                built.makespan <= scheduler.plan_greedy().makespan)
        r.check("one plan per drone, ids 1..N",
                [plan.drone_id for plan in built.plans] == [1, 2])

    def _format_problem(self, graph: Graph, schedule: Schedule) -> str:
        """Check the output lines against the format of the subject.

        Returns:
            A description of the first problem, or "".
        """
        last: dict[int, str] = {}
        for number, line in enumerate(schedule.lines(), start=1):
            seen: set[int] = set()
            for token in line.split():
                found = re.fullmatch(r"D(\d+)-(\S+)", token)
                if found is None:
                    return f"turn {number}: bad token {token!r}"
                drone = int(found.group(1))
                if drone in seen:
                    return f"turn {number}: D{drone} appears twice"
                seen.add(drone)
                last[drone] = found.group(2)
        for drone in range(1, graph.nb_drones + 1):
            if last.get(drone) != graph.end.name:
                return f"D{drone} does not finish on {graph.end.name!r}"
        return ""

    def _map_files(self) -> None:
        """The scheduler on the provided maps."""
        r = self.runner
        if not self.root.is_dir():
            r.note(f"folder {self.root}/ not found: skipped")
            return
        for path in sorted(self.root.rglob("*.txt")):
            known = path.name in PathfindingChecks.START_DISTANCES
            if "invalid" in path.relative_to(self.root).parts or not known:
                continue
            label = str(path.relative_to(self.root))
            try:
                graph = MapParser().parse_file(str(path))
            except FlyInError:
                continue
            scheduler = Scheduler(graph)
            schedule = scheduler.build()
            greedy = scheduler.plan_greedy()
            r.check(f"{label}: one plan per drone",
                    [plan.drone_id for plan in schedule.plans]
                    == list(range(1, graph.nb_drones + 1)))
            problem = PlanAuditor.replay(graph, schedule.plans)
            r.check(f"{label}: the table accepts every plan",
                    problem == "", problem)
            problem = self._format_problem(graph, schedule)
            r.check(f"{label}: output format and every drone delivered",
                    problem == "", problem)
            r.check(f"{label}: one line per turn ({schedule.makespan})",
                    len(schedule.lines()) == schedule.makespan)
            r.check(f"{label}: build() is never worse than greedy",
                    schedule.makespan <= greedy.makespan,
                    f"{schedule.makespan} > {greedy.makespan}")
            target = SpaceTimeChecks.TARGETS.get(path.name)
            if target is not None:
                r.check(f"{label}: {schedule.makespan} turns <= {target}",
                        schedule.makespan <= target)
            optimum = SpaceTimeChecks.OPTIMUM.get(path.name)
            if optimum is not None:
                r.check(f"{label}: optimal makespan {optimum}",
                        schedule.makespan == optimum,
                        f"got {schedule.makespan}")
            if path.name == SpaceTimeChecks.RECORD_NAME:
                r.note(f"{label}: {schedule.makespan} turns "
                       f"(record to beat: {SpaceTimeChecks.RECORD})")


class SimulatorChecks:
    """Checks for simulator.py: the independent referee of the output.

    The simulator only reads the TEXT of the output. These checks feed
    it valid outputs and falsified ones.
    """

    LINE: list[str] = [
        "start_hub: s 0 0", "hub: a 1 0", "hub: b 2 0", "end_hub: g 3 0",
        "connection: s-a", "connection: a-b", "connection: b-g",
    ]
    GOOD: list[str] = ["D1-a", "D1-b D2-a", "D1-g D2-b", "D2-g"]
    ZONE_CAPACITY: list[str] = [
        "start_hub: s 0 0", "hub: a 1 0", "end_hub: g 2 0",
        "connection: s-a [max_link_capacity=2]",
        "connection: a-g [max_link_capacity=2]",
    ]
    LINK_CAPACITY: list[str] = [
        "start_hub: s 0 0", "hub: a 1 0 [max_drones=2]", "end_hub: g 2 0",
        "connection: s-a", "connection: a-g [max_link_capacity=2]",
    ]
    SWAP: list[str] = [
        "start_hub: s 0 0", "hub: x 1 0", "hub: y 2 0", "end_hub: g 3 0",
        "connection: s-x", "connection: x-y", "connection: y-g",
    ]
    FLIGHT: list[str] = [
        "start_hub: s 0 0", "hub: x 1 0 [zone=blocked]",
        "hub: r 1 1 [zone=restricted]", "hub: n 2 1", "end_hub: g 3 0",
        "connection: s-x", "connection: x-g", "connection: s-r",
        "connection: r-g", "connection: s-n", "connection: n-g",
    ]

    def __init__(self, runner: CheckRunner, root: Path) -> None:
        """Store the runner and the folder of the maps."""
        self.runner: CheckRunner = runner
        self.root: Path = root

    def run(self) -> None:
        """Run every simulator check."""
        self.runner.section("simulator.py - Simulator")
        self.runner.guard("independence", self._independence)
        self.runner.guard("parse_line", self._parse)
        self.runner.guard("valid outputs", self._valid)
        self.runner.guard("rejected outputs", self._rejected)
        self.runner.guard("provided maps", self._map_files)

    def _independence(self) -> None:
        """The referee must not share code with the planners."""
        module = sys.modules[Simulator.__module__]
        source = Path(str(module.__file__)).read_text(encoding="utf-8")
        forbidden = re.findall(
            r"^\s*(?:from|import)\s+(plan|reservation|scheduler|pathfinding"
            r"|space_time_astar)\b", source, re.M)
        self.runner.check(
            "simulator.py imports none of the planning modules",
            not forbidden, f"it imports {sorted(set(forbidden))}")

    def _graph(self, lines: list[str], drones: int) -> Graph:
        """Parse a small map for ``drones`` drones."""
        return MapParser().parse_lines([f"nb_drones: {drones}"] + lines)

    def _parse(self) -> None:
        """Reading one output line."""
        r = self.runner
        simulator = Simulator(self._graph(self.LINE, 2))
        moves = simulator.parse_line(1, "D1-roof1 D2-corridorA")
        r.check("two tokens: drone ids and targets",
                [(m.drone_id, m.target) for m in moves]
                == [(1, "roof1"), (2, "corridorA")]
                and not any(m.is_flight for m in moves))
        flight = simulator.parse_line(1, "D3-roof1-roof2")
        r.check("a connection name is a flight: D3-roof1-roof2",
                len(flight) == 1 and flight[0].is_flight
                and flight[0].drone_id == 3
                and flight[0].target == "roof1-roof2")
        r.check("an empty line has no move",
                simulator.parse_line(1, "") == [])
        for bad in ("robot-a", "D1", "D1-", "D01-a", "D1-a-b-c", "d1-a"):
            r.expect_error(f"bad token {bad!r} is rejected",
                           partial(simulator.parse_line, 1, bad),
                           SimulationError)

    def _valid(self) -> None:
        """Valid outputs and their metrics."""
        r = self.runner
        result = Simulator(self._graph(self.LINE, 2)).run(self.GOOD)
        r.check("2 drones in a line: 4 turns", result.turns == 4)
        r.check("arrival turns are D1: 3, D2: 4",
                result.arrival_turns == {1: 3, 2: 4},
                str(result.arrival_turns))
        r.check("moves per turn are 1, 2, 2, 1",
                result.moves_per_turn == [1, 2, 2, 1],
                str(result.moves_per_turn))
        r.check("average turns per drone is 3.5",
                result.average_turns == 3.5)
        r.check("total cost is 6", result.total_cost == 6)
        r.check("history: one state per turn, positions after turn 2",
                len(result.history) == 4
                and result.history[1].positions == {1: "b", 2: "a"},
                str(result.history[1].positions if result.history else ""))
        r.check("a drone entering a zone another one leaves is accepted",
                result.history[2].positions == {1: "g", 2: "b"})

        flight = Simulator(self._graph(self.FLIGHT, 1)).run(
            ["D1-s-r", "D1-r", "D1-g"])
        r.check("a flight: 3 turns, cost 2 + 1, arrival turn 3",
                flight.turns == 3 and flight.total_cost == 3
                and flight.arrival_turns == {1: 3})
        r.check("in flight the position is the connection s-r",
                flight.history[0].positions == {1: "s-r"},
                str(flight.history[0].positions))
        again = Simulator(self._graph(self.LINE, 2))
        first = again.run(self.GOOD)
        second = again.run(self.GOOD)
        r.check("a simulator can be run twice",
                first.turns == second.turns == 4)

    def _cases(self) -> list[tuple[str, list[str], list[str], int | None]]:
        """Falsified outputs: (label, map, lines, turn that must be named).

        An error about one turn must contain ``turn N``. Errors about
        the whole output (a drone never delivered) name no turn.
        """
        line, flight = self.LINE, self.FLIGHT
        two_in_a = ["D1-a D2-a", "D1-b", "D1-g D2-b", "D2-g"]
        return [
            ("a zone of capacity 1 gets 2 drones",
             self.ZONE_CAPACITY, two_in_a, 1),
            ("a link of capacity 1 is used twice",
             self.LINK_CAPACITY, ["D1-a D2-a", "D1-g D2-g"], 1),
            ("two drones swap places on a link of capacity 1", self.SWAP,
             ["D1-x", "D1-y D2-x", "D1-x D2-y", "D1-y D2-x",
              "D1-g D2-y", "D2-g"], 3),
            ("teleport from s to g", line, ["D1-g"], 1),
            ("move to a zone that is not a neighbour", line, ["D1-b"], 1),
            ("unknown zone", line, ["D1-zz"], 1),
            ("unknown drone D9", line, ["D9-a"], 1),
            ("a drone moves twice in a turn", line, ["D1-a D1-b"], 1),
            ("bad token", line, ["robot-a"], 1),
            ("a drone moves after being delivered", line,
             self.GOOD[:3] + ["D2-g D1-g"], 4),
            ("an extra empty turn after the end", line,
             self.GOOD + [""], 5),
            ("a drone is never delivered (last line missing)", line,
             self.GOOD[:-1], None),
            ("no line at all", line, [], None),
            ("entering a blocked zone", flight, ["D1-x", "D1-g"], 1),
            ("a restricted zone entered like a normal one", flight,
             ["D1-r", "D1-g"], 1),
            ("a flight toward a NORMAL zone", flight,
             ["D1-s-n", "D1-n", "D1-g"], 1),
            ("a flight that does not land on the next turn", flight,
             ["D1-s-r", "", "D1-g"], 2),
            ("a flight that lands on the wrong zone", flight,
             ["D1-s-r", "D1-g"], 2),
            ("a flight leaving from the wrong zone", flight,
             ["D1-n-r", "D1-r", "D1-g"], 1),
        ]

    def _rejected(self) -> None:
        """Every falsified output must raise SimulationError."""
        r = self.runner
        for label, map_lines, output, turn in self._cases():
            drones = 2 if map_lines is not self.FLIGHT else 1
            graph = self._graph(map_lines, drones)
            try:
                Simulator(graph).run(output)
            except SimulationError as error:
                named = turn is None or f"turn {turn}" in str(error)
                r.check(f"{label}: rejected", named,
                        f"the message should name turn {turn}: {error}")
            except Exception as exc:
                r.check(f"{label}: rejected", False,
                        f"CRASH {type(exc).__name__}: {exc}")
            else:
                r.check(f"{label}: rejected", False, "it was ACCEPTED")

    def _map_files(self) -> None:
        """Replay the scheduler output of the provided maps."""
        r = self.runner
        if not self.root.is_dir():
            r.note(f"folder {self.root}/ not found: skipped")
            return
        for path in sorted(self.root.rglob("*.txt")):
            known = path.name in PathfindingChecks.START_DISTANCES
            if "invalid" in path.relative_to(self.root).parts or not known:
                continue
            label = str(path.relative_to(self.root))
            try:
                graph = MapParser().parse_file(str(path))
            except FlyInError:
                continue
            schedule = Scheduler(graph).build()
            lines = schedule.lines()
            result = Simulator(graph).run(lines)
            r.check(f"{label}: the referee accepts the output "
                    f"({result.turns} turns)",
                    result.turns == schedule.makespan)
            r.check(f"{label}: same arrival turns as the plans",
                    result.arrival_turns == {
                        plan.drone_id: plan.arrival_turn
                        for plan in schedule.plans})
            r.check(f"{label}: same total cost "
                    f"({result.total_cost}) as the plans",
                    result.total_cost == schedule.total_cost,
                    f"{result.total_cost} != {schedule.total_cost}")
            counted = [len(line.split()) for line in lines]
            r.check(f"{label}: moves per turn = tokens per line",
                    result.moves_per_turn == counted)
            r.expect_error(f"{label}: without its last line, rejected",
                           partial(Simulator(graph).run, lines[:-1]),
                           SimulationError)
            r.expect_error(f"{label}: with an extra turn, rejected",
                           partial(Simulator(graph).run, lines + [""]),
                           SimulationError)


class MapFileChecks:
    """Parse every map found in a folder (and its sub-folders)."""

    EXPECTED: dict[str, tuple[int, int, int]] = {
        "01_linear_path.txt": (2, 4, 3),
        "02_simple_fork.txt": (4, 5, 5),
        "03_basic_capacity.txt": (4, 4, 3),
        "01_dead_end_trap.txt": (5, 6, 5),
        "02_circular_loop.txt": (6, 7, 7),
        "03_priority_puzzle.txt": (5, 7, 7),
        "01_maze_nightmare.txt": (8, 17, 22),
        "02_capacity_hell.txt": (12, 15, 21),
        "03_ultimate_challenge.txt": (15, 31, 37),
        "01_the_impossible_dream.txt": (25, 54, 70),
    }

    def __init__(self, runner: CheckRunner, root: Path) -> None:
        """Store the runner and the folder to scan."""
        self.runner: CheckRunner = runner
        self.root: Path = root

    def run(self) -> None:
        """Parse every *.txt file under the root folder."""
        self.runner.section(f"map files in {self.root}/")
        if not self.root.is_dir():
            self.runner.note("folder not found: skipped")
            return
        files = sorted(self.root.rglob("*.txt"))
        if not files:
            self.runner.note("no .txt file found: skipped")
            return
        for path in files:
            label = str(path.relative_to(self.root))
            if "invalid" in path.relative_to(self.root).parts:
                self._expect_rejected(path, label)
            else:
                self._expect_parsed(path, label)

    def _expect_parsed(self, path: Path, label: str) -> None:
        """A normal map must parse (and match its expected counts)."""
        r = self.runner
        try:
            graph = MapParser().parse_file(str(path))
        except FlyInError as error:
            r.check(label, False, str(error))
            return
        counts = (graph.nb_drones, len(graph.zones), len(graph.connections))
        expected = self.EXPECTED.get(path.name)
        text = f"{label}  ({counts[0]} drones, {counts[1]} zones, "
        text += f"{counts[2]} links, reachable={graph.is_goal_reachable()})"
        if expected is None:
            r.check(text, True)
        else:
            r.check(text, counts == expected,
                    f"expected {expected}, got {counts}")

    def _expect_rejected(self, path: Path, label: str) -> None:
        """A map stored in an 'invalid' folder must be rejected."""
        r = self.runner
        try:
            MapParser().parse_file(str(path))
        except FlyInError as error:
            r.check(f"{label}  (rejected as expected)", True)
            r.note(str(error))
        else:
            r.check(label, False, "the invalid map was ACCEPTED")


class MapSummary:
    """Print a readable summary of one map."""

    def show(self, path: str) -> int:
        """Parse a map and print its con tent.

        Args:
            path: Path of the map file.

        Returns:
            0 on success, 1 if the map cannot be parsed.
        """
        try:
            graph = MapParser().parse_file(path)
        except FlyInError as error:
            print(error, file=sys.stderr)
            return 1
        kinds: dict[str, int] = {}
        for zone in graph.zones:
            kinds[zone.zone_type.value] = kinds.get(
                zone.zone_type.value, 0) + 1
        print(f"map          : {path}")
        print(f"drones       : {graph.nb_drones}")
        print(f"zones        : {len(graph.zones)}  {kinds}")
        print(f"connections  : {len(graph.connections)}")
        print(f"start -> end : {graph.start.name} -> {graph.end.name}")
        print(f"bounds       : {graph.bounds()}  (min_x, min_y, max_x, max_y)")
        print(f"reachable    : {graph.is_goal_reachable()}")
        print("\nZONES")
        for zone in graph.zones:
            cap = "inf" if zone.capacity is None else str(zone.capacity)
            role = "START" if zone.is_start else "END" if zone.is_end else ""
            print(f"  {zone.name:<22} ({zone.x:>3},{zone.y:>3}) "
                  f"{zone.zone_type.value:<10} cap={cap:<4} "
                  f"{zone.color or '-':<10} {role}")
        print("\nCONNECTIONS")
        for link in graph.connections:
            print(f"  {link.zone_a.name} <-> {link.zone_b.name}"
                  f"  cap={link.max_link_capacity}")
        return 0


class App:
    """Entry point of the check script."""

    def run(self, argv: list[str]) -> int:
        """Run the requested action.

        Args:
            argv: Command line arguments (without the program name).

        Returns:
            Process exit code.
        """
        if argv:
            return MapSummary().show(argv[0])
        runner = CheckRunner()
        ModelChecks(runner).run()
        GraphChecks(runner).run()
        ParserChecks(runner).run()
        PathfindingChecks(runner, Path("maps")).run()
        PlanChecks(runner).run()
        ReservationChecks(runner).run()
        SpaceTimeChecks(runner, Path("maps")).run()
        SchedulerChecks(runner, Path("maps")).run()
        SimulatorChecks(runner, Path("maps")).run()
        MapFileChecks(runner, Path("maps")).run()
        return runner.summary()


if __name__ == "__main__":
    sys.exit(App().run(sys.argv[1:]))
