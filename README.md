*This project has been created as part of the 42 curriculum by harenaan.*

# Fly-in

## Description

Fly-in moves a fleet of drones from a start zone to an end zone through a
network of connected zones, in as few turns as possible. Zones and
connections have capacities, some zones are slower or forbidden, and the
program must never break one of these rules.

The program reads a map file, plans every drone, checks its own plan with a
separate referee, prints one line per turn, and (if you want) plays the
result in a small pygame window.

Rules in short:

- One turn moves every drone at most one step.
- A normal or priority zone costs 1 turn to enter, a restricted zone costs 2
  (the drone is on the connection during the first turn), a blocked zone
  cannot be entered.
- A zone holds `max_drones` drones at the end of a turn (default 1). The start
  and end zones have no limit.
- A connection carries `max_link_capacity` drones per turn, in both
  directions together (default 1).

## Instructions

Python 3.10 or newer is needed.

```
make install                         # creates .venv, installs pygame-ce, flake8, mypy
make run MAP=maps/easy/01_linear_path.txt
python3 fly_in.py maps/hard/02_capacity_hell.txt
```

Options of `fly_in.py`:

| option        | effect                                                     |
|---------------|------------------------------------------------------------|
| `--no-window` | print the turns only, open no window                       |
| `-q`          | do not print the statistics (they go to stderr anyway)     |

The turns are written on stdout, the statistics and the errors on stderr.
Without pygame, or without a display, the program still prints the turns and
only warns that no window was opened.

Other targets:

| command            | what it does                                       |
|--------------------|----------------------------------------------------|
| `make install`     | create `.venv` and install the dependencies        |
| `make run MAP=...` | run the program on a map                           |
| `make debug`       | run under `pdb`                                    |
| `make lint`        | flake8 and mypy                                    |
| `make lint-strict` | flake8 and mypy `--strict`                         |
| `make clean`       | remove the caches                                  |
| `make fclean`      | `clean`, and remove `.venv`                        |

All the targets use the python of `.venv`, so there is nothing to activate.
To work in the venv from your own shell: `source .venv/bin/activate`.

### Map format

```
nb_drones: 4
start_hub: start 0 0 [color=green]
hub: corridor 1 0 [zone=restricted max_drones=2 color=orange]
end_hub: goal 3 0 [color=red]
connection: start-corridor [max_link_capacity=2]
connection: corridor-goal
```

`nb_drones` comes first. Zone names have no space and no dash. Metadata are
optional: `zone` (normal, blocked, restricted, priority), `color` (one word),
`max_drones`, and `max_link_capacity` for connections. A `#` starts a comment.
Every error stops the program with a message naming the line.

### Example

Input (`maps/easy/01_linear_path.txt`, 2 drones on a line of 4 zones):

```
nb_drones: 2
start_hub: start 0 0 [color=green]
hub: waypoint1 1 0 [color=blue]
hub: waypoint2 2 0 [color=blue]
end_hub: goal 3 0 [color=red]
connection: start-waypoint1
connection: waypoint1-waypoint2
connection: waypoint2-goal
```

Output (one line per turn, a drone that does not move is not written):

```
D1-waypoint1
D1-waypoint2 D2-waypoint1
D1-goal D2-waypoint2
D2-goal
```

With a restricted zone `r` between `s` and `e`, a drone first shows the
connection it is flying on, then the zone it lands in:

```
D1-s-r
D1-r
D1-e
```

## Algorithm

**Graph.** Written by hand (`graph.py`): a dictionary of zones, a dictionary of
connections keyed by the unordered pair of names, and an adjacency list.
Blocked zones are hidden from `neighbors()`, so no search can step on them.

**Distances.** `DistanceMap` runs a Dijkstra backwards from the end zone, with
the entry cost of each zone as edge weight. It gives, for every zone, the least
number of turns to the goal if nobody was in the way. It is the heuristic of
the search, and it is also how we detect a map where the goal cannot be
reached.

**One drone: space-time A\*.** A state is `(zone, turn)`. From a state a drone
can wait, or move to a neighbour if the reservation table allows it. The
priority of a state is `turn + distance to goal` (never too optimistic, so the
first goal found is a fastest one). Among candidates with the same priority, a second
score breaks the tie: a wait or a normal move adds 1, a restricted move adds
2, a move into a priority zone adds 0. That is how priority zones are
preferred without ever making a drone slower.

**Capacities: the reservation table.** For every turn it counts the drones in
each zone (at the end of the turn) and on each connection (during the whole
move, so a restricted move uses the connection for 2 turns, and two drones
swapping places on a capacity-1 connection are refused). A drone in flight is
in no zone.

**The fleet: prioritized planning.** Drone 1 is planned on an empty table, its
plan is reserved, drone 2 plans around it, and so on. A turn limit of
`rank * distance(start, goal)` always leaves a solution because a drone can
wait on the unlimited start zone. This greedy pass is fast but not always
optimal, so a short improvement pass follows: up to 3 rounds, each one
removes the drone that arrives last and one other drone, plans them again
with the last one first, and keeps the result only if it is shorter (or as
short but cheaper).

**Referee.** `simulator.py` does not know the planner. It reads the printed
lines as text and replays them, rule by rule, and raises an error on the first
broken rule. The program only prints a plan once the referee accepted it.

**Complexity.** The distance map costs O((Z + E) log Z). One A\* explores at
most Z x T states (T is the turn limit) and looks at the neighbours of each, so
O(Z x T x deg x log(Z x T)). The fleet costs D of those searches, plus at most
3 x (D - 1) x 2 searches for the improvement pass.

**Trade-offs.** Prioritized planning is not guaranteed optimal: the first
drones are never disturbed by the last ones, except in the improvement pass.
In exchange it is fast (every provided map solves in well under a second) and
easy to reason about. In practice it beats every reference value (see below).

### Results

| map                      | drones | turns | reference |
|--------------------------|-------:|------:|----------:|
| easy / linear path       |      2 |     4 |         6 |
| easy / simple fork       |      4 |     4 |         6 |
| easy / basic capacity    |      4 |     4 |         8 |
| medium / dead end trap   |      5 |     8 |        15 |
| medium / circular loop   |      6 |    15 |        20 |
| medium / priority puzzle |      5 |     7 |        12 |
| hard / maze nightmare    |      8 |    13 |        45 |
| hard / capacity hell     |     12 |    16 |        60 |
| hard / ultimate challenge|     15 |    26 |        35 |
| challenger / impossible dream | 25 |   43 |        45 |

## Visual representation

The window is drawn with pygame and shows the checked simulation, so what you
see is exactly what was printed.

- Each zone uses the colour given in its metadata (`rainbow` is animated,
  unknown words get a fallback colour, colours too dark to see are
  replaced by a grey).
- The shape tells the type: circle for normal, diamond for restricted, circle
  with a gold ring for priority, dark disc with a cross for blocked.
- `S` and `E` mark the start and end zones, and a number inside a zone is its
  `max_drones` when it is above 1.
- Connections get thicker with their capacity and light up while a drone uses
  them.
- Drones are arrows coloured by number, turned towards where they go, and they
  glide between two turns. A restricted move is seen halfway on the connection.
  When more than six drones share a zone, a counter is added next to it.
- The top line shows the turn, the number of moves and how many drones are
  delivered.

Keys: `SPACE` pause, `LEFT`/`RIGHT` step one turn, `UP`/`DOWN` faster or slower,
`R` restart, `N` zone names, `I` drone numbers, `ESC` or `Q` quit.

## Project structure

| file                  | role                                              |
|-----------------------|---------------------------------------------------|
| `fly_in.py`           | entry point, command line, ties everything together |
| `map_parser.py`       | reads and validates the map file                  |
| `models.py`           | `ZoneType`, `Zone`, `Connection`, `Drone`         |
| `graph.py`            | graph structure                                   |
| `pathfinding.py`      | distances to the goal                             |
| `space_time_a_star.py`| single-drone search                               |
| `reservation.py`      | who is where, turn by turn                        |
| `plan.py`             | `Leg` and `DronePlan`: what one drone does        |
| `scheduler.py`        | plans the fleet and improves the result           |
| `simulator.py`        | independent referee                               |
| `renderer.py`         | pygame window                                     |
| `errors.py`           | the project's exceptions                          |

## Resources

- David Silver, *Cooperative Pathfinding*, AIIDE 2005: the idea of planning
  agents one after the other on a space-time reservation table.
- Dijkstra and A\* in any algorithms course.
- pygame-ce documentation: https://pyga.me/docs/
