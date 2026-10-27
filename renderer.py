"""Pygame window of the Fly-in project: draws the map and the drones."""

import math
import os

from errors import DisplayError
from graph import Graph
from models import Zone
from simulator import SimulationResult

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame  # noqa: E402


class ColorResolver:
    """Turn the colour words of the map into pygame colours."""

    NEUTRAL: pygame.Color = pygame.Color(110, 120, 135)
    MIN_BRIGHTNESS: int = 70
    DARK_REPLACEMENT: pygame.Color = pygame.Color(90, 90, 90)
    FALLBACK: tuple[pygame.Color, ...] = (
        pygame.Color(86, 180, 233), pygame.Color(240, 228, 66),
        pygame.Color(0, 158, 115), pygame.Color(213, 94, 0),
        pygame.Color(204, 121, 167), pygame.Color(230, 159, 0))
    DRONES: tuple[pygame.Color, ...] = (
        pygame.Color(255, 214, 10), pygame.Color(64, 224, 208),
        pygame.Color(255, 105, 180), pygame.Color(124, 252, 0),
        pygame.Color(255, 140, 0), pygame.Color(135, 206, 250),
        pygame.Color(238, 130, 238), pygame.Color(255, 99, 71))

    def resolve(self, color: str | None, phase: float = 0.0) -> pygame.Color:
        """Colour of a zone; unknown or too dark words get a safe one."""
        if color is None:
            return pygame.Color(self.NEUTRAL)
        name = color.lower()
        if name == "rainbow":
            found = self._rainbow(phase)
        else:
            try:
                found = pygame.Color(name)
            except ValueError:
                index = sum(ord(c) for c in name) % len(self.FALLBACK)
                found = pygame.Color(self.FALLBACK[index])
        if max(found.r, found.g, found.b) >= self.MIN_BRIGHTNESS:
            return found
        return pygame.Color(self.DARK_REPLACEMENT)

    def drone_color(self, drone_id: int) -> pygame.Color:
        """Colour of a drone, cycling over a short palette."""
        return pygame.Color(self.DRONES[(drone_id - 1) % len(self.DRONES)])

    def _rainbow(self, phase: float) -> pygame.Color:
        """Colour of the ``rainbow`` word, moving with the animation."""
        color = pygame.Color(0, 0, 0)
        color.hsva = ((phase * 60.0) % 360.0, 75.0, 95.0, 100.0)
        return color


class Viewport:
    """Convert map coordinates into window pixels."""

    MARGIN: int = 70
    MAX_SCALE: float = 110.0

    def __init__(self, graph: Graph, size: tuple[int, int]) -> None:
        """Fit the whole map inside a window of ``size`` pixels."""
        self._size: tuple[int, int] = size
        min_x, min_y, max_x, max_y = graph.bounds()
        span_x, span_y = max_x - min_x, max_y - min_y
        usable_w = size[0] - 2 * self.MARGIN
        usable_h = size[1] - 2 * self.MARGIN
        candidates = [self.MAX_SCALE]
        if span_x > 0:
            candidates.append(usable_w / span_x)
        if span_y > 0:
            candidates.append(usable_h / span_y)
        self._scale: float = min(candidates)
        self._left: float = self.MARGIN + (usable_w - span_x * self._scale) / 2
        self._top: float = self.MARGIN + (usable_h - span_y * self._scale) / 2
        self._min_x: int = min_x
        self._max_y: int = max_y

    @property
    def size(self) -> tuple[int, int]:
        """Size of the window in pixels."""
        return self._size

    @property
    def scale(self) -> float:
        """Pixels per map unit."""
        return self._scale

    @property
    def radius(self) -> int:
        """Radius of a zone circle, in pixels."""
        return max(6, min(22, int(self._scale * 0.28)))

    def point(self, zone: Zone) -> tuple[float, float]:
        """Pixel position of a zone."""
        return (self._left + (zone.x - self._min_x) * self._scale,
                self._top + (self._max_y - zone.y) * self._scale)


class PlaybackController:
    """Time of the animation: pause, speed, step by step."""

    SPEEDS: tuple[float, ...] = (0.5, 1.0, 2.0, 4.0, 8.0)

    def __init__(self, turns: int, show_names: bool = True) -> None:
        """Start at turn 0 for a simulation of ``turns`` turns."""
        self.turns: int = turns
        self.position: float = 0.0
        self.paused: bool = False
        self.show_names: bool = show_names
        self.show_ids: bool = False
        self._speed_index: int = 2

    @property
    def speed(self) -> float:
        """Current speed, in turns per second."""
        return self.SPEEDS[self._speed_index]

    @property
    def turn(self) -> int:
        """Last turn fully played."""
        return min(int(self.position), self.turns)

    @property
    def fraction(self) -> float:
        """Progress inside the turn being played (0 to 1)."""
        if self.turn >= self.turns:
            return 0.0
        return self.position - self.turn

    @property
    def finished(self) -> bool:
        """Tell whether the animation reached the end."""
        return self.position >= self.turns

    def update(self, dt: float) -> None:
        """Move the clock forward by ``dt`` seconds."""
        if not self.paused and not self.finished:
            self.position = min(float(self.turns),
                                self.position + self.speed * dt)

    def toggle_pause(self) -> None:
        """Pause or resume; restart if the animation is over."""
        if self.finished:
            self.restart()
        else:
            self.paused = not self.paused

    def step(self, delta: int) -> None:
        """Pause and jump ``delta`` turns forward or back."""
        self.paused = True
        target = int(round(self.position)) + delta
        self.position = float(max(0, min(self.turns, target)))

    def faster(self) -> None:
        """Go to the next speed."""
        self._speed_index = min(len(self.SPEEDS) - 1, self._speed_index + 1)

    def slower(self) -> None:
        """Go to the previous speed."""
        self._speed_index = max(0, self._speed_index - 1)

    def restart(self) -> None:
        """Go back to turn 0 and play."""
        self.position = 0.0
        self.paused = False

    def toggle_names(self) -> None:
        """Show or hide the zone names."""
        self.show_names = not self.show_names

    def toggle_ids(self) -> None:
        """Show or hide the drone numbers."""
        self.show_ids = not self.show_ids


class PygameRenderer:
    """Draws the map and animates the drones."""

    DEFAULT_SIZE: tuple[int, int] = (1100, 700)
    BACKGROUND: pygame.Color = pygame.Color(24, 27, 34)
    LINK_COLOR: pygame.Color = pygame.Color(78, 86, 100)
    ACTIVE_LINK_COLOR: pygame.Color = pygame.Color(210, 215, 228)
    OUTLINE: pygame.Color = pygame.Color(235, 235, 240)
    PRIORITY_RING: pygame.Color = pygame.Color(255, 200, 40)
    TEXT: pygame.Color = pygame.Color(205, 210, 220)
    HINT: pygame.Color = pygame.Color(130, 138, 152)
    DRONE_OUTLINE: pygame.Color = pygame.Color(20, 20, 20)
    MAX_RING: int = 6
    BACKGROUND_IMAGE: str | None = None
    BACKGROUND_DIM: int = 150

    def __init__(
        self,
        graph: Graph,
        result: SimulationResult,
        size: tuple[int, int] = DEFAULT_SIZE,
        background_image: str | None = BACKGROUND_IMAGE,
    ) -> None:
        """Prepare the window for a graph and a checked simulation."""
        self._graph: Graph = graph
        self._result: SimulationResult = result
        self._viewport: Viewport = Viewport(graph, size)
        self._colors: ColorResolver = ColorResolver()
        self._controller: PlaybackController = PlaybackController(
            result.turns, show_names=len(graph.zones) <= 20)
        self._fonts: dict[int, pygame.font.Font] = {}
        self._background: pygame.Surface | None = self._load_background(
            background_image, size)

    @staticmethod
    def _load_background(
        path: str | None, size: tuple[int, int]
    ) -> pygame.Surface | None:
        """Load, fit and darken the background image (None if unusable)."""
        if path is None:
            return None
        try:
            image = pygame.image.load(path)
        except (pygame.error, OSError):
            return None
        image = pygame.transform.scale(image, size)
        shade = pygame.Surface(size)
        shade.set_alpha(PygameRenderer.BACKGROUND_DIM)
        image.blit(shade, (0, 0))
        return image

    def drone_point(
        self, drone_id: int, position: float
    ) -> tuple[float, float]:
        """Pixel position of a drone at a moment of the animation."""
        return self._frame_slots(position)[drone_id]

    def drone_heading(self, drone_id: int, position: float) -> float:
        """Direction of a drone, in radians (screen axes)."""
        turn = min(int(position), self._result.turns)
        if turn < self._result.turns:
            before = self._slots(turn)[drone_id]
            after = self._slots(turn + 1)[drone_id]
            dx, dy = after[0] - before[0], after[1] - before[1]
            if math.hypot(dx, dy) > 1e-6:
                return math.atan2(dy, dx)
        x, y = self.drone_point(drone_id, position)
        end_x, end_y = self._viewport.point(self._graph.end)
        if math.hypot(end_x - x, end_y - y) > 1e-6:
            return math.atan2(end_y - y, end_x - x)
        return -math.pi / 2

    def draw(self, surface: pygame.Surface, position: float) -> None:
        """Draw one frame."""
        if self._background is not None:
            surface.blit(self._background, (0, 0))
        else:
            surface.fill(self.BACKGROUND)
        self._draw_links(surface, self._active_links(position))
        self._draw_zones(surface, position)
        self._draw_drones(surface, position)
        self._draw_hud(surface, position)

    @staticmethod
    def link_width(capacity: int) -> int:
        """Line width of a connection: wider means more capacity."""
        return 1 + 2 * min(capacity, 3)

    def run(self, max_frames: int | None = None) -> None:
        """Open the window and play until the user quits."""
        size = self._viewport.size
        try:
            pygame.display.init()
            pygame.font.init()
            surface = pygame.display.set_mode(size)
        except pygame.error as error:
            raise DisplayError(f"cannot open a window: {error}") from error
        pygame.display.set_caption(
            f"Fly-in - {self._graph.nb_drones} drones, "
            f"{self._result.turns} turns")
        clock = pygame.time.Clock()
        frames = 0
        running = True
        while running and (max_frames is None or frames < max_frames):
            dt = clock.tick(60) / 1000.0
            running = self._handle_events()
            self._controller.update(dt)
            self.draw(surface, self._controller.position)
            pygame.display.flip()
            frames += 1
        self._fonts.clear()
        pygame.font.quit()
        pygame.display.quit()

    def _positions(self, turn: int) -> dict[int, str]:
        """Where every drone is at the end of ``turn``."""
        if turn == 0:
            start = self._graph.start.name
            return {n: start for n in range(1, self._graph.nb_drones + 1)}
        return self._result.history[turn - 1].positions

    def _anchor(self, place: str) -> tuple[float, float]:
        """Pixel of a zone, or the middle of a connection in flight."""
        if "-" in place:
            first, second = place.split("-")
            ax, ay = self._viewport.point(self._graph.get_zone(first))
            bx, by = self._viewport.point(self._graph.get_zone(second))
            return ((ax + bx) / 2, (ay + by) / 2)
        return self._viewport.point(self._graph.get_zone(place))

    def _slots(self, turn: int) -> dict[int, tuple[float, float]]:
        """Spread the drones sharing a place so they do not overlap."""
        groups: dict[str, list[int]] = {}
        for drone_id, place in sorted(self._positions(turn).items()):
            groups.setdefault(place, []).append(drone_id)
        slots: dict[int, tuple[float, float]] = {}
        ring = self._viewport.radius * 0.6
        for place, ids in groups.items():
            cx, cy = self._anchor(place)
            for rank, drone_id in enumerate(ids):
                if 1 < len(ids) <= self.MAX_RING:
                    angle = -math.pi / 2 + 2 * math.pi * rank / len(ids)
                    slots[drone_id] = (cx + ring * math.cos(angle),
                                       cy + ring * math.sin(angle))
                else:
                    slots[drone_id] = (cx, cy)
        return slots

    def _frame_slots(self, position: float) -> dict[int, tuple[float, float]]:
        """Drone pixels between two turns (linear move)."""
        turn = min(int(position), self._result.turns)
        fraction = 0.0 if turn >= self._result.turns else position - turn
        before = self._slots(turn)
        if fraction == 0.0:
            return before
        after = self._slots(turn + 1)
        return {d: (before[d][0] + (after[d][0] - before[d][0]) * fraction,
                    before[d][1] + (after[d][1] - before[d][1]) * fraction)
                for d in before}

    def _active_links(self, position: float) -> set[frozenset[str]]:
        """Connections used by a move during the current turn."""
        turn = min(int(position), self._result.turns)
        if turn >= self._result.turns or position == turn:
            return set()
        before, after = self._positions(turn), self._positions(turn + 1)
        active: set[frozenset[str]] = set()
        for drone_id, place in before.items():
            other = after[drone_id]
            if place == other:
                continue
            flight = place if "-" in place else other
            names = flight.split("-") if "-" in flight else [place, other]
            active.add(frozenset(names))
        return active

    def _font(self, size: int) -> pygame.font.Font:
        """Font of a given size, created once."""
        if size not in self._fonts:
            self._fonts[size] = pygame.font.Font(None, size)
        return self._fonts[size]

    def _text(self, surface: pygame.Surface, text: str,
              center: tuple[float, float], size: int,
              color: pygame.Color) -> None:
        """Write a centred text."""
        image = self._font(size).render(text, True, color)
        x, y = int(center[0]), int(center[1])
        surface.blit(image, image.get_rect(center=(x, y)))

    def _draw_links(self, surface: pygame.Surface,
                    active: set[frozenset[str]]) -> None:
        """Draw the connections."""
        for link in self._graph.connections:
            color = (self.ACTIVE_LINK_COLOR if link.key in active
                     else self.LINK_COLOR)
            pygame.draw.line(
                surface, color, self._viewport.point(link.zone_a),
                self._viewport.point(link.zone_b),
                self.link_width(link.max_link_capacity))

    def _draw_zones(self, surface: pygame.Surface, position: float) -> None:
        """Draw the zones with their colour and shape."""
        radius = self._viewport.radius
        black = pygame.Color(0, 0, 0)
        for zone in self._graph.zones:
            cx, cy = self._viewport.point(zone)
            fill = self._colors.resolve(zone.color, position)
            if zone.is_blocked:
                fill = fill.lerp(black, 0.6)
            if zone.is_restricted:
                reach = radius * 1.2
                points = [(cx - reach, cy), (cx, cy - reach),
                          (cx + reach, cy), (cx, cy + reach)]
                pygame.draw.polygon(surface, fill, points)
                pygame.draw.polygon(surface, self.OUTLINE, points, 2)
            else:
                pygame.draw.circle(surface, fill, (cx, cy), radius)
                ring = self.PRIORITY_RING if zone.is_priority else self.OUTLINE
                pygame.draw.circle(surface, ring, (cx, cy), radius,
                                   4 if zone.is_priority else 2)
            if zone.is_start or zone.is_end:
                pygame.draw.circle(
                    surface, self.OUTLINE, (cx, cy), radius + 5, 2)
            self._draw_zone_mark(surface, zone, (cx, cy), radius)
            if self._controller.show_names:
                size = max(12, min(16, int(self._viewport.scale * 0.2)))
                self._text(surface, zone.name, (cx, cy + radius + 12),
                           size, self.TEXT)

    def _draw_zone_mark(self, surface: pygame.Surface, zone: Zone,
                        center: tuple[float, float], radius: int) -> None:
        """Write S, E, a cross or the capacity inside a zone."""
        cx, cy = center
        if zone.is_blocked:
            d = radius * 0.6
            pygame.draw.line(surface, self.OUTLINE,
                             (cx - d, cy - d), (cx + d, cy + d), 3)
            pygame.draw.line(surface, self.OUTLINE,
                             (cx - d, cy + d), (cx + d, cy - d), 3)
        elif zone.is_start:
            self._text(surface, "S", center, radius + 4, self.OUTLINE)
        elif zone.is_end:
            self._text(surface, "E", center, radius + 4, self.OUTLINE)
        elif zone.max_drones > 1:
            self._text(surface, str(zone.max_drones), center,
                       radius + 2, self.OUTLINE)

    @staticmethod
    def _drone_polygon(
        center: tuple[float, float], heading: float, size: float
    ) -> list[tuple[float, float]]:
        """Arrow-head polygon of a drone (3 points)."""
        reach = size * 1.6
        return [(center[0] + reach * math.cos(heading + offset),
                 center[1] + reach * math.sin(heading + offset))
                for offset in (0.0, 2.5, -2.5)]

    def _draw_drones(self, surface: pygame.Surface, position: float) -> None:
        """Draw the drones and the counters of crowded zones."""
        slots = self._frame_slots(position)
        size = max(4, int(self._viewport.radius * 0.35))
        shown = min(self._result.turns, int(round(position)))
        counts: dict[str, int] = {}
        for place in self._positions(shown).values():
            counts[place] = counts.get(place, 0) + 1
        radius = self._viewport.radius
        for zone in self._graph.zones:
            if counts.get(zone.name, 0) > self.MAX_RING:
                cx, cy = self._viewport.point(zone)
                self._text(surface, str(counts[zone.name]),
                           (cx + radius * 0.9, cy - radius * 0.9), 20,
                           self.OUTLINE)
        for drone_id, (x, y) in slots.items():
            heading = self.drone_heading(drone_id, position)
            pygame.draw.polygon(surface, self.DRONE_OUTLINE,
                                self._drone_polygon((x, y), heading, size + 1))
            pygame.draw.polygon(surface, self._colors.drone_color(drone_id),
                                self._drone_polygon((x, y), heading, size))
            if self._controller.show_ids:
                self._text(surface, str(drone_id), (x + size + 8, y), 14,
                           self.TEXT)

    def _draw_hud(self, surface: pygame.Surface, position: float) -> None:
        """Draw the turn counter and the key hints."""
        turn = min(int(position), self._result.turns)
        moves = 0 if turn == 0 else self._result.history[turn - 1].moves
        delivered = sum(1 for t in self._result.arrival_turns.values()
                        if t <= turn)
        line = (f"Turn {turn} / {self._result.turns}    moves {moves}    "
                f"delivered {delivered}/{self._graph.nb_drones}")
        surface.blit(self._font(28).render(line, True, self.TEXT), (20, 18))
        hint = ("SPACE pause   LEFT/RIGHT step   UP/DOWN speed   "
                "R restart   N names   I ids   ESC quit")
        height = self._viewport.size[1]
        surface.blit(self._font(18).render(hint, True, self.HINT),
                     (20, height - 28))

    def _handle_events(self) -> bool:
        """Handle the keyboard; return False to quit."""
        keys = self._controller
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type != pygame.KEYDOWN:
                continue
            if event.key in (pygame.K_ESCAPE, pygame.K_q):
                return False
            actions = {
                pygame.K_SPACE: keys.toggle_pause,
                pygame.K_RIGHT: lambda: keys.step(1),
                pygame.K_LEFT: lambda: keys.step(-1),
                pygame.K_UP: keys.faster, pygame.K_DOWN: keys.slower,
                pygame.K_r: keys.restart, pygame.K_n: keys.toggle_names,
                pygame.K_i: keys.toggle_ids,
            }
            if event.key in actions:
                actions[event.key]()
        return True
