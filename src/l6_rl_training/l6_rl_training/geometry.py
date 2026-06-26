from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Tuple

import numpy as np


@dataclass(frozen=True)
class BoxObstacle:
    name: str
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]

    def active_at_z(self, z: float) -> bool:
        half_z = 0.5 * self.size[2]
        return self.center[2] - half_z <= z <= self.center[2] + half_z


@dataclass(frozen=True)
class Scenario:
    name: str
    goal_xy: Tuple[float, float]
    obstacles: Tuple[BoxObstacle, ...]
    t_max_s: float


@dataclass
class DistanceQuery:
    name: str
    signed_distance: float
    normal: np.ndarray


def with_room_boundaries(obstacles: Iterable[BoxObstacle]) -> Tuple[BoxObstacle, ...]:
    room = [
        BoxObstacle("l2_wall_x_positive", (5.00, 0.00, 1.50), (0.12, 8.00, 3.00)),
        BoxObstacle("l2_wall_x_negative", (-5.00, 0.00, 1.50), (0.12, 8.00, 3.00)),
        BoxObstacle("l2_wall_y_positive", (0.00, 4.00, 1.50), (10.00, 0.12, 3.00)),
        BoxObstacle("l2_wall_y_negative", (0.00, -4.00, 1.50), (10.00, 0.12, 3.00)),
    ]
    return tuple(list(obstacles) + room)


SCENARIOS = {
    "S1_single_front_obstacle": Scenario(
        "S1_single_front_obstacle",
        (4.20, 0.00),
        with_room_boundaries([
            BoxObstacle("front_block", (3.00, 0.00, 1.25), (0.70, 1.60, 2.50)),
        ]),
        90.0,
    ),
    "S2_narrow_gate": Scenario(
        "S2_narrow_gate",
        (4.20, 0.00),
        with_room_boundaries([
            BoxObstacle("gate_left_pillar", (2.50, 0.90, 1.30), (0.45, 0.45, 2.60)),
            BoxObstacle("gate_right_pillar", (2.50, -0.90, 1.30), (0.45, 0.45, 2.60)),
            BoxObstacle("gate_top_reference", (2.50, 0.00, 2.65), (0.50, 2.25, 0.12)),
        ]),
        100.0,
    ),
    "S3_corridor": Scenario(
        "S3_corridor",
        (4.20, 0.00),
        with_room_boundaries([
            BoxObstacle("corridor_left_wall", (2.20, 1.20, 1.35), (5.80, 0.16, 2.70)),
            BoxObstacle("corridor_right_wall", (2.20, -1.20, 1.35), (5.80, 0.16, 2.70)),
        ]),
        110.0,
    ),
    "S4_table_or_low_obstacle": Scenario(
        "S4_table_or_low_obstacle",
        (4.20, 0.00),
        with_room_boundaries([
            BoxObstacle("low_table_top", (2.60, 0.00, 0.80), (1.70, 1.10, 0.16)),
            BoxObstacle("low_table_leg_1", (1.90, 0.45, 0.40), (0.12, 0.12, 0.80)),
            BoxObstacle("low_table_leg_2", (3.30, 0.45, 0.40), (0.12, 0.12, 0.80)),
            BoxObstacle("low_table_leg_3", (1.90, -0.45, 0.40), (0.12, 0.12, 0.80)),
            BoxObstacle("low_table_leg_4", (3.30, -0.45, 0.40), (0.12, 0.12, 0.80)),
        ]),
        100.0,
    ),
    "S5_corner": Scenario(
        "S5_corner",
        (3.80, 2.60),
        with_room_boundaries([
            BoxObstacle("corner_vertical_wall", (2.00, -0.70, 1.35), (0.18, 2.60, 2.70)),
            BoxObstacle("corner_horizontal_wall", (3.05, 1.30, 1.35), (2.30, 0.18, 2.70)),
            BoxObstacle("corner_inner_block", (2.55, 0.55, 1.25), (0.55, 0.55, 2.50)),
        ]),
        120.0,
    ),
}


def query_box_2d(point_xy: np.ndarray, box: BoxObstacle) -> DistanceQuery:
    center = np.array(box.center[:2], dtype=np.float64)
    half = 0.5 * np.array(box.size[:2], dtype=np.float64)
    delta = point_xy - center
    abs_delta = np.abs(delta)
    q = abs_delta - half
    outside = np.maximum(q, 0.0)
    outside_dist = float(np.linalg.norm(outside))

    if outside_dist > 1e-9:
        closest = np.minimum(np.maximum(point_xy, center - half), center + half)
        normal = point_xy - closest
        n = np.linalg.norm(normal)
        if n < 1e-9:
            normal = np.array([1.0, 0.0], dtype=np.float64)
        else:
            normal = normal / n
        return DistanceQuery(box.name, outside_dist, normal)

    px = half[0] - abs_delta[0]
    py = half[1] - abs_delta[1]
    if px <= py:
        normal = np.array([1.0 if delta[0] >= 0.0 else -1.0, 0.0], dtype=np.float64)
        return DistanceQuery(box.name, -float(px), normal)
    normal = np.array([0.0, 1.0 if delta[1] >= 0.0 else -1.0], dtype=np.float64)
    return DistanceQuery(box.name, -float(py), normal)


def nearest_obstacle_2d(point_xy: np.ndarray, scenario: Scenario, flight_z: float = 2.0) -> DistanceQuery:
    best = DistanceQuery("none", math.inf, np.array([1.0, 0.0], dtype=np.float64))
    for obstacle in scenario.obstacles:
        if not obstacle.active_at_z(flight_z):
            continue
        q = query_box_2d(point_xy, obstacle)
        if q.signed_distance < best.signed_distance:
            best = q
    return best


def ray_distance_2d(
    point_xy: np.ndarray,
    direction_xy: np.ndarray,
    scenario: Scenario,
    flight_z: float,
    max_range: float,
    step: float = 0.08,
) -> float:
    direction_xy = direction_xy / max(np.linalg.norm(direction_xy), 1e-9)
    distance = 0.0
    while distance <= max_range:
        probe = point_xy + direction_xy * distance
        if nearest_obstacle_2d(probe, scenario, flight_z).signed_distance <= 0.0:
            return distance
        distance += step
    return max_range


def active_obstacles(scenario: Scenario, flight_z: float = 2.0) -> List[BoxObstacle]:
    return [obstacle for obstacle in scenario.obstacles if obstacle.active_at_z(flight_z)]
