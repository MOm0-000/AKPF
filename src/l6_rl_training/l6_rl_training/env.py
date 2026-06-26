from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np

from .geometry import SCENARIOS, DistanceQuery, nearest_obstacle_2d, ray_distance_2d


class L6AkpfEnv:
    """Small Gym-style 2D training environment for L6 AKPF feature experiments."""

    def __init__(
        self,
        scenario: str = "S1_single_front_obstacle",
        observation_mode: str = "akpf",
        use_shield: bool = False,
        seed: Optional[int] = None,
    ) -> None:
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario: {scenario}")
        if observation_mode not in {"baseline", "akpf", "full"}:
            raise ValueError("observation_mode must be baseline, akpf, or full")
        self.scenario_name = scenario
        self.scenario = SCENARIOS[scenario]
        self.observation_mode = observation_mode
        self.use_shield = use_shield
        self.rng = np.random.default_rng(seed)

        self.dt = 0.20
        self.flight_z = 2.0
        self.max_xy_speed = 0.50
        self.max_accel_alpha = 0.45
        self.sensor_range = 4.0
        self.body_radius = 0.25
        self.safety_margin = 0.15
        self.goal_radius = 0.50
        self.caution_d_eff = 0.85
        self.stop_d_eff = 0.08
        self.toward_speed_gain = 0.80
        self.max_steps = int(math.ceil(self.scenario.t_max_s / self.dt))
        self.ray_angles = np.linspace(-math.pi, math.pi, 16, endpoint=False)

        self.pos = np.zeros(2, dtype=np.float64)
        self.vel = np.zeros(2, dtype=np.float64)
        self.prev_action = np.zeros(2, dtype=np.float64)
        self.step_count = 0
        self.prev_goal_dist = 0.0
        self.min_clearance = math.inf
        self.last_shield_active = False

    @property
    def observation_size(self) -> int:
        return int(self._make_obs().shape[0])

    @property
    def action_size(self) -> int:
        return 2

    def reset(self, seed: Optional[int] = None) -> np.ndarray:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.pos = self.rng.normal(loc=0.0, scale=0.05, size=2).astype(np.float64)
        self.vel = np.zeros(2, dtype=np.float64)
        self.prev_action = np.zeros(2, dtype=np.float64)
        self.step_count = 0
        self.prev_goal_dist = self._goal_dist()
        self.min_clearance = math.inf
        self.last_shield_active = False
        return self._make_obs()

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, float]]:
        raw_action = np.asarray(action, dtype=np.float64)
        raw_action = np.clip(raw_action, -1.0, 1.0)
        cmd = raw_action * self.max_xy_speed
        nearest_before = self._nearest()
        if self.use_shield:
            cmd = self._shield_velocity(cmd, nearest_before)
        else:
            self.last_shield_active = False

        self.vel = (1.0 - self.max_accel_alpha) * self.vel + self.max_accel_alpha * cmd
        self.pos = self.pos + self.vel * self.dt
        self.step_count += 1

        nearest = self._nearest()
        self.min_clearance = min(self.min_clearance, nearest.signed_distance)
        goal_dist = self._goal_dist()
        progress = self.prev_goal_dist - goal_dist
        self.prev_goal_dist = goal_dist

        collision = nearest.signed_distance <= self.body_radius
        reached = goal_dist <= self.goal_radius
        timeout = self.step_count >= self.max_steps
        done = collision or reached or timeout

        smooth = float(np.linalg.norm(raw_action - self.prev_action))
        self.prev_action = raw_action
        d_eff = self._effective_distance(nearest)
        risk_penalty = max(0.0, 0.20 - d_eff)

        reward = 6.0 * progress - 0.02 * smooth - 0.20 * risk_penalty
        if reached:
            reward += 60.0
        if collision:
            reward -= 60.0
        if timeout:
            reward -= 20.0

        info = {
            "goal_dist": float(goal_dist),
            "nearest": float(nearest.signed_distance),
            "d_eff": float(d_eff),
            "collision": float(collision),
            "reached": float(reached),
            "timeout": float(timeout),
            "shield_active": float(self.last_shield_active),
            "min_clearance": float(self.min_clearance),
        }
        return self._make_obs(), float(reward), bool(done), info

    def teacher_action(self) -> np.ndarray:
        """L3-inspired AKPF candidate action used only for L6 warm-start baselines."""
        nearest = self._nearest()
        d_eff = self._effective_distance(nearest)
        target = self._teacher_target()
        nav_error = target - self.pos
        goal_dist = np.linalg.norm(nav_error)
        goal_dir = nav_error / max(goal_dist, 1e-9)

        if d_eff < 0.35:
            away = nearest.normal / max(np.linalg.norm(nearest.normal), 1e-9)
            left = np.array([-away[1], away[0]], dtype=np.float64)
            right = -left
            tangent = left if np.dot(left, goal_dir) >= np.dot(right, goal_dir) else right
            risk = np.clip((0.35 - d_eff) / 0.35, 0.0, 1.0)
            direction = away * (0.35 + 0.45 * risk) + tangent * (0.95 - 0.30 * risk)
            direction = direction / max(np.linalg.norm(direction), 1e-9)
            return np.clip(direction * 0.44 / self.max_xy_speed, -1.0, 1.0).astype(np.float32)

        angles = [-180, -150, -120, -90, -60, -35, -15, 0, 15, 35, 60, 90, 120, 150, 180]
        current_margin = nearest.signed_distance - self.body_radius - self.safety_margin
        progress_floor = 0.05 if current_margin < 0.18 else -0.05

        candidates = []
        for angle in angles:
            rad = math.radians(angle)
            c = math.cos(rad)
            s = math.sin(rad)
            direction = np.array([
                goal_dir[0] * c - goal_dir[1] * s,
                goal_dir[0] * s + goal_dir[1] * c,
            ], dtype=np.float64)
            margin = self._predicted_clearance_margin(direction, goal_dist)
            progress = float(np.dot(direction, goal_dir))
            candidates.append((direction, margin, progress))

        has_safe_progress = any(progress >= progress_floor and margin >= 0.15 for _, margin, progress in candidates)
        has_noncolliding_progress = any(progress >= progress_floor and margin >= 0.0 for _, margin, progress in candidates)

        best_dir = goal_dir
        best_score = -math.inf
        for direction, margin, progress in candidates:
            if has_safe_progress and (progress < progress_floor or margin < 0.15):
                continue
            if not has_safe_progress and has_noncolliding_progress and (progress < progress_floor or margin < 0.0):
                continue
            safety_score = min(max(margin, -1.0), 0.50)
            smoothness = float(np.dot(direction, self.vel / max(np.linalg.norm(self.vel), 1e-9))) if np.linalg.norm(self.vel) > 1e-6 else 0.0
            score = 1.10 * safety_score + 2.40 * progress + 0.20 * smoothness
            if score > best_score:
                best_score = score
                best_dir = direction

        speed = self.max_xy_speed
        if goal_dist < 1.20:
            speed *= np.clip(goal_dist / 1.20, 0.30, 1.0)
        return np.clip(best_dir * speed / self.max_xy_speed, -1.0, 1.0).astype(np.float32)

    def _teacher_target(self) -> np.ndarray:
        goal = np.asarray(self.scenario.goal_xy, dtype=np.float64)
        if self._segment_clearance_margin(self.pos, goal) >= 0.15:
            return goal

        global_dir = goal - self.pos
        global_dir = global_dir / max(np.linalg.norm(global_dir), 1e-9)
        inflate = self.body_radius + self.safety_margin + 0.25
        best = goal
        best_cost = math.inf
        found = False
        for obstacle in self.scenario.obstacles:
            if obstacle.name.startswith("l2_wall_") or not obstacle.active_at_z(self.flight_z):
                continue
            center = np.asarray(obstacle.center[:2], dtype=np.float64)
            half = 0.5 * np.asarray(obstacle.size[:2], dtype=np.float64)
            for sx in (-1.0, 1.0):
                for sy in (-1.0, 1.0):
                    candidate = center + np.array([sx * (half[0] + inflate), sy * (half[1] + inflate)])
                    if self._point_clearance_margin(candidate) < 0.05:
                        continue
                    if self._segment_clearance_margin(self.pos, candidate) < 0.0:
                        continue
                    candidate_vec = candidate - self.pos
                    progress_distance = float(np.dot(candidate_vec, global_dir))
                    if progress_distance < 0.20:
                        continue
                    goal_visible = self._segment_clearance_margin(candidate, goal) >= 0.15
                    cost = (
                        np.linalg.norm(candidate - self.pos)
                        + np.linalg.norm(goal - candidate)
                        - 0.45 * progress_distance
                        - (0.50 if goal_visible else 0.0)
                    )
                    if cost < best_cost:
                        best = candidate
                        best_cost = float(cost)
                        found = True
        return best if found else goal

    def _goal_vec(self) -> np.ndarray:
        return np.asarray(self.scenario.goal_xy, dtype=np.float64) - self.pos

    def _goal_dist(self) -> float:
        return float(np.linalg.norm(self._goal_vec()))

    def _nearest(self) -> DistanceQuery:
        return nearest_obstacle_2d(self.pos, self.scenario, self.flight_z)

    def _effective_distance(self, nearest: DistanceQuery) -> float:
        v_toward = max(0.0, -float(np.dot(self.vel, nearest.normal)))
        d_brake = (v_toward * v_toward) / max(2.0 * 0.70, 1e-6)
        d_delay = v_toward * 0.25
        return nearest.signed_distance - self.body_radius - self.safety_margin - d_brake - d_delay

    def _shield_velocity(self, cmd: np.ndarray, nearest: DistanceQuery) -> np.ndarray:
        d_eff = self._effective_distance(nearest)
        speed = float(np.linalg.norm(cmd))
        if speed > self.max_xy_speed:
            cmd = cmd * (self.max_xy_speed / max(speed, 1e-9))

        self.last_shield_active = d_eff < self.caution_d_eff
        toward = max(0.0, -float(np.dot(cmd, nearest.normal)))
        max_toward = self.toward_speed_gain * max(0.0, d_eff - self.stop_d_eff)
        if toward > max_toward:
            cmd = cmd + nearest.normal * (toward - max_toward)
            self.last_shield_active = True
        return cmd

    def _predicted_clearance_margin(self, direction: np.ndarray, goal_distance: float) -> float:
        direction = direction / max(np.linalg.norm(direction), 1e-9)
        min_clearance = math.inf
        for horizon in [0.45, 0.90, 1.35, 1.80, 2.40, 3.00]:
            sample_distance = min(horizon, max(0.25, goal_distance))
            probe = self.pos + direction * sample_distance
            q = nearest_obstacle_2d(probe, self.scenario, self.flight_z)
            min_clearance = min(min_clearance, q.signed_distance)
        return min_clearance - self.body_radius - self.safety_margin

    def _point_clearance_margin(self, point_xy: np.ndarray) -> float:
        q = nearest_obstacle_2d(point_xy, self.scenario, self.flight_z)
        return q.signed_distance - self.body_radius - self.safety_margin

    def _segment_clearance_margin(self, start_xy: np.ndarray, end_xy: np.ndarray) -> float:
        delta = end_xy - start_xy
        length = float(np.linalg.norm(delta))
        if length < 1e-6:
            return self._point_clearance_margin(start_xy)
        direction = delta / length
        steps = max(2, int(math.ceil(length / 0.20)))
        margin = math.inf
        for i in range(steps + 1):
            probe = start_xy + direction * (length * i / steps)
            margin = min(margin, self._point_clearance_margin(probe))
        return margin

    def _baseline_obs(self) -> np.ndarray:
        goal = self._goal_vec()
        goal_dist = np.linalg.norm(goal)
        goal_dir = goal / max(goal_dist, 1e-9)
        rays = []
        for angle in self.ray_angles:
            direction = np.array([math.cos(angle), math.sin(angle)], dtype=np.float64)
            rays.append(ray_distance_2d(self.pos, direction, self.scenario, self.flight_z, self.sensor_range))
        ray_obs = np.asarray(rays, dtype=np.float64) / self.sensor_range
        return np.concatenate([
            goal_dir,
            np.array([goal_dist / self.sensor_range], dtype=np.float64),
            self.vel / self.max_xy_speed,
            ray_obs,
        ])

    def _akpf_obs(self) -> np.ndarray:
        goal = self._goal_vec()
        goal_dist = np.linalg.norm(goal)
        goal_dir = goal / max(goal_dist, 1e-9)
        nav_target = self._teacher_target()
        nav_error = nav_target - self.pos
        nav_dist = np.linalg.norm(nav_error)
        nav_dir = nav_error / max(nav_dist, 1e-9)
        nearest = self._nearest()
        d_eff = self._effective_distance(nearest)
        risk = np.clip((1.20 - d_eff) / 1.20, 0.0, 1.0)
        attractive = goal_dir
        repulsive = nearest.normal * risk * risk
        tangent_left = np.array([-nearest.normal[1], nearest.normal[0]], dtype=np.float64)
        tangent_right = -tangent_left
        tangent = tangent_left if np.dot(tangent_left, goal_dir) >= np.dot(tangent_right, goal_dir) else tangent_right
        shield_flag = 1.0 if self.last_shield_active else 0.0
        return np.concatenate([
            goal_dir,
            np.array([goal_dist / self.sensor_range], dtype=np.float64),
            nav_dir,
            np.array([nav_dist / self.sensor_range], dtype=np.float64),
            self.vel / self.max_xy_speed,
            np.array([
                nearest.signed_distance / self.sensor_range,
                d_eff / self.sensor_range,
                risk,
            ], dtype=np.float64),
            nearest.normal,
            attractive,
            repulsive,
            tangent,
            np.array([shield_flag], dtype=np.float64),
        ])

    def _make_obs(self) -> np.ndarray:
        if self.observation_mode == "baseline":
            obs = self._baseline_obs()
        elif self.observation_mode == "akpf":
            obs = self._akpf_obs()
        else:
            obs = np.concatenate([self._baseline_obs(), self._akpf_obs()])
        return obs.astype(np.float32)
