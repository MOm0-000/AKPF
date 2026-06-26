from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
import torch

from .env import L6AkpfEnv
from .geometry import SCENARIOS, active_obstacles
from .train_ppo import ActorCritic


def rollout_policy(policy_path: Path, scenario: str, seed: int):
    checkpoint = torch.load(policy_path, map_location="cpu")
    config = checkpoint["config"]
    model = ActorCritic(checkpoint["obs_size"], checkpoint["action_size"])
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    env = L6AkpfEnv(
        scenario=scenario,
        observation_mode=config["obs_mode"],
        use_shield=config["use_shield"],
        seed=seed,
    )
    obs = env.reset(seed=seed)
    trajectory = [env.pos.copy()]
    info = {}
    done = False
    while not done:
        with torch.no_grad():
            mean, _ = model(torch.as_tensor(obs[None, :], dtype=torch.float32))
        obs, _, done, info = env.step(np.clip(mean.numpy()[0], -1.0, 1.0))
        trajectory.append(env.pos.copy())
    return np.asarray(trajectory), info


def draw_scenario(ax, scenario_name: str, trajectory: np.ndarray, info: dict) -> None:
    scenario = SCENARIOS[scenario_name]
    for obstacle in active_obstacles(scenario, flight_z=2.0):
        cx, cy = obstacle.center[:2]
        sx, sy = obstacle.size[:2]
        color = "#555555" if obstacle.name.startswith("l2_wall_") else "#b95c3b"
        alpha = 0.20 if obstacle.name.startswith("l2_wall_") else 0.55
        rect = Rectangle((cx - sx / 2.0, cy - sy / 2.0), sx, sy, color=color, alpha=alpha)
        ax.add_patch(rect)

    goal = np.asarray(scenario.goal_xy)
    ax.add_patch(Circle(goal, 0.50, color="#2f8f5b", alpha=0.18))
    ax.scatter([goal[0]], [goal[1]], marker="*", s=130, color="#1f7a45", label="goal")
    ax.plot(trajectory[:, 0], trajectory[:, 1], color="#1f5fbf", linewidth=2.2)
    ax.scatter([trajectory[0, 0]], [trajectory[0, 1]], color="#222222", s=35, label="start")
    ax.scatter([trajectory[-1, 0]], [trajectory[-1, 1]], color="#1f5fbf", s=35, label="end")
    status = "GOAL" if info.get("reached", 0.0) else "FAIL"
    ax.set_title(
        f"{scenario_name}\n{status} goal={info.get('goal_dist', 0.0):.2f} clear={info.get('min_clearance', 0.0):.2f}",
        fontsize=9,
    )
    ax.set_xlim(-1.2, 5.2)
    ax.set_ylim(-3.3, 3.5)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)


def visualize(policy_path: Path, output: Path, seed: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 5, figsize=(18, 4.2), constrained_layout=True)
    for ax, scenario in zip(axes, SCENARIOS):
        trajectory, info = rollout_policy(policy_path, scenario, seed)
        draw_scenario(ax, scenario, trajectory, info)
    fig.suptitle("L6 simplified AKPF+Shield policy trajectories", fontsize=13)
    fig.savefig(output, dpi=180)
    print(f"saved_figure={output}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Render L6 policy trajectories for S1-S5.")
    parser.add_argument("--policy", required=True)
    parser.add_argument("--output", default="/tmp/l6_policy_trajectories.png")
    parser.add_argument("--seed", type=int, default=1000)
    args = parser.parse_args()
    visualize(Path(args.policy), Path(args.output), args.seed)


if __name__ == "__main__":
    main()
