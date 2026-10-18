"""Single-stroke vector lettering for the Drawbot PCB demonstration."""

from __future__ import annotations

from math import cos, radians, sin
from typing import TypeAlias

Point2D: TypeAlias = tuple[float, float]
Stroke = tuple[Point2D, ...]

# Normalized single-stroke glyphs. Coordinates span a 1 x 1 cell, with Y up.
_GLYPHS: dict[str, tuple[Stroke, ...]] = {
    "A": (
        ((0.0, 0.0), (0.5, 1.0), (1.0, 0.0)),
        ((0.22, 0.42), (0.78, 0.42)),
    ),
    "G": (
        ((0.95, 0.72), (0.85, 0.95), (0.15, 0.95), (0.05, 0.82),
         (0.05, 0.18), (0.18, 0.05), (0.82, 0.05), (0.95, 0.18),
         (0.95, 0.52), (0.55, 0.52)),
    ),
    "L": (
        ((0.12, 1.0), (0.12, 0.0), (0.95, 0.0)),
    ),
    "M": (
        ((0.05, 0.0), (0.05, 1.0), (0.5, 0.48), (0.95, 1.0), (0.95, 0.0)),
    ),
    "N": (
        ((0.08, 0.0), (0.08, 1.0), (0.92, 0.0), (0.92, 1.0)),
    ),
    "O": (
        ((0.18, 0.0), (0.05, 0.14), (0.05, 0.86), (0.18, 1.0),
         (0.82, 1.0), (0.95, 0.86), (0.95, 0.14), (0.82, 0.0),
         (0.18, 0.0)),
    ),
    "S": (
        ((0.95, 0.9), (0.15, 0.9), (0.05, 0.78), (0.05, 0.58),
         (0.15, 0.52), (0.85, 0.46), (0.95, 0.38), (0.95, 0.16),
         (0.85, 0.08), (0.05, 0.08)),
    ),
    "T": (
        ((0.02, 0.92), (0.98, 0.92)),
        ((0.5, 0.92), (0.5, 0.0)),
    ),
}


def make_word_paths(
    word: str,
    center_y: float,
    letter_width: float = 0.015,
    letter_height: float = 0.020,
    spacing: float = 0.003,
    orientation_degrees: float = 0.0,
) -> tuple[Stroke, ...]:
    """Return centered XY strokes, optionally rotated around the PCB origin."""
    normalized_word = word.upper()
    if not normalized_word:
        raise ValueError("word must not be empty")
    unknown_letters = set(normalized_word) - _GLYPHS.keys()
    if unknown_letters:
        raise ValueError(f"unsupported letters: {', '.join(sorted(unknown_letters))}")
    if letter_width <= 0.0 or letter_height <= 0.0 or spacing < 0.0:
        raise ValueError("letter dimensions must be positive and spacing non-negative")

    total_width = len(normalized_word) * letter_width + (len(normalized_word) - 1) * spacing
    first_x = -total_width / 2.0
    paths: list[Stroke] = []
    for index, letter in enumerate(normalized_word):
        letter_left = first_x + index * (letter_width + spacing)
        for stroke in _GLYPHS[letter]:
            paths.append(
                tuple(
                    (
                        letter_left + x * letter_width,
                        center_y + (y - 0.5) * letter_height,
                    )
                    for x, y in stroke
                )
            )
    angle = radians(orientation_degrees)
    cosine = cos(angle)
    sine = sin(angle)
    return tuple(
        tuple(
            (x * cosine - y * sine, x * sine + y * cosine)
            for x, y in stroke
        )
        for stroke in paths
    )


PCB_GREETING_PATHS = (
    make_word_paths(
        "SALAMA",
        center_y=0.022,
        letter_width=0.017,
        letter_height=0.014,
        spacing=0.0025,
        orientation_degrees=-90.0,
    ),
    make_word_paths(
        "TONGASOA",
        center_y=-0.022,
        letter_width=0.017,
        letter_height=0.014,
        spacing=0.0025,
        orientation_degrees=-90.0,
    ),
)
