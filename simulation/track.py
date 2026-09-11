"""
track.py - NXP Cup track geometry.

The track is described as a sequence of segments (Straight / Curve).
World coordinates: X=east, Y=north, heading in radians (0 = east).

Public API:
    get_track_centerline()       -> list of (x, y) in metres
    get_track_boundaries()       -> (left_pts, right_pts)
    find_closest_point(x, y)     -> (closest_pt, heading_rad, cte)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

from config import TRACK_WIDTH


# ---------------------------------------------------------------------------
# Segment primitives
# ---------------------------------------------------------------------------

@dataclass
class Straight:
    length_m: float


@dataclass
class Curve:
    """
    radius_m : turning radius of the centreline (metres)
    angle_deg: arc swept.  Positive = left turn, negative = right turn.
    """
    radius_m: float
    angle_deg: float


# ---------------------------------------------------------------------------
# Sample NXP Cup-like track definition
# ---------------------------------------------------------------------------
# Approximate dimensions of an NXP Cup arena (~6 m x 4 m inner loop).

TRACK_SEGMENTS = [
    # Start straight
    Straight(1.15),
    
    # Chicane right-left
    Curve(0.60, -45),
    Curve(0.60, 45),
    
    # Approach to intersection
    Straight(0.65),
    
    # The big figure-8 Left loop (270 degrees)
    # Starts facing East, ends facing South
    Curve(1.5, 270),
    
    # Intersection straight (crosses the start straight)
    Straight(0.65),
    
    # Chicane left-right
    Curve(0.60, 45),
    Curve(0.60, -45),
    
    # Approach to second loop
    Straight(1.15),
    
    # The big figure-8 Right loop (270 degrees)
    # Starts facing South, ends facing East (back at start)
    Curve(1.5, -270),
]


# ---------------------------------------------------------------------------
# Centerline generation
# ---------------------------------------------------------------------------

def _build_centerline(
    segments: list,
    start_x: float = 0.0,
    start_y: float = 0.0,
    start_heading: float = 0.0,  # radians
    points_per_metre: float = 20.0,
) -> List[Tuple[float, float, float]]:
    """
    Returns list of (x, y, heading_rad) for each point along the centreline.
    heading_rad is the tangent direction (forward direction of the track).
    """
    pts: List[Tuple[float, float, float]] = [(start_x, start_y, start_heading)]
    x, y, h = start_x, start_y, start_heading

    for seg in segments:
        if isinstance(seg, Straight):
            n = max(2, int(seg.length_m * points_per_metre))
            dx = seg.length_m / n
            for _ in range(n):
                x += dx * math.cos(h)
                y += dx * math.sin(h)
                pts.append((x, y, h))

        elif isinstance(seg, Curve):
            angle_rad = math.radians(seg.angle_deg)
            arc_len = abs(seg.radius_m * angle_rad)
            n = max(4, int(arc_len * points_per_metre))
            dangle = angle_rad / n
            for _ in range(n):
                # move forward by the arc sub-step, then rotate
                step = abs(seg.radius_m * dangle)
                x += step * math.cos(h + dangle / 2.0)
                y += step * math.sin(h + dangle / 2.0)
                h += dangle
                pts.append((x, y, h))

    return pts


# Build once at import time
_CENTRELINE_FULL: List[Tuple[float, float, float]] = _build_centerline(TRACK_SEGMENTS)
_CENTRELINE_XY: List[Tuple[float, float]] = [(p[0], p[1]) for p in _CENTRELINE_FULL]


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------

def get_track_centerline() -> List[Tuple[float, float]]:
    """Return list of (x, y) world-coordinate points along the track centre."""
    return list(_CENTRELINE_XY)


def get_track_boundaries() -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    """
    Return (left_boundary, right_boundary).
    Each boundary is a list of (x, y) points offset TRACK_WIDTH/2 from centre.
    Left = positive lateral direction (left hand side looking forward).
    Right = negative lateral direction.
    """
    half = TRACK_WIDTH / 2.0
    left_pts: List[Tuple[float, float]] = []
    right_pts: List[Tuple[float, float]] = []

    for (cx, cy, h) in _CENTRELINE_FULL:
        # Perpendicular: rotate heading +90 deg for left, -90 for right
        lx = cx + half * math.cos(h + math.pi / 2.0)
        ly = cy + half * math.sin(h + math.pi / 2.0)
        rx = cx + half * math.cos(h - math.pi / 2.0)
        ry = cy + half * math.sin(h - math.pi / 2.0)
        left_pts.append((lx, ly))
        right_pts.append((rx, ry))

    return left_pts, right_pts


def find_closest_point(
    x: float, y: float
) -> Tuple[Tuple[float, float], float, float]:
    """
    Find the closest point on the track centreline to (x, y).

    Returns:
        closest_pt  : (cx, cy) world coordinates of nearest centreline point
        heading_rad : track tangent direction at that point
        cte         : signed cross-track error in metres.
                      Positive = car is to the RIGHT of the centreline.
                      Negative = car is to the LEFT.
    """
    best_dist_sq = float("inf")
    best_idx = 0

    for i, (cx, cy) in enumerate(_CENTRELINE_XY):
        d2 = (x - cx) ** 2 + (y - cy) ** 2
        if d2 < best_dist_sq:
            best_dist_sq = d2
            best_idx = i

    cx, cy, h = _CENTRELINE_FULL[best_idx]

    # Signed CTE: project car offset onto the right-pointing lateral vector
    dx = x - cx
    dy = y - cy
    # right-pointing unit vector perpendicular to heading
    rx = math.cos(h - math.pi / 2.0)
    ry = math.sin(h - math.pi / 2.0)
    cte = dx * rx + dy * ry  # positive = car is to the right

    return (cx, cy), h, cte
