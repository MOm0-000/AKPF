from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch

from .env import L6AkpfEnv
from .geometry import SCENARIOS
from .train_ppo import ActorCritic


def evaluate(policy_path: Path, episodes_per_scenario: int) -> None:
    checkpoint = torch.load(policy_path, map_location="cpu")
    config = checkpoint["config"]
    model = ActorCritic(checkpoint["obs_size"], checkpoint["action_size"])
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    print("scenario,episodes,success_rate,collision_rate,timeout_rate,mean_goal_dist,mean_min_clearance")
    for scenario in SCENARIOS:
        rows = []
        for ep in range(episodes_per_scenario):
            env = L6AkpfEnv(
                scenario=scenario,
                observation_mode=config["obs_mode"],
                use_shield=config["use_shield"],
                seed=1000 + ep,
            )
            obs = env.reset()
            done = False
            info = {}
            while not done:
                with torch.no_grad():
                    mean, _ = model(torch.as_tensor(obs[None, :], dtype=torch.float32))
                obs, _, done, info = env.step(np.clip(mean.numpy()[0], -1.0, 1.0))
            rows.append(info)
        print(
            f"{scenario},{episodes_per_scenario},"
            f"{np.mean([r['reached'] for r in rows]):.4f},"
            f"{np.mean([r['collision'] for r in rows]):.4f},"
            f"{np.mean([r['timeout'] for r in rows]):.4f},"
            f"{np.mean([r['goal_dist'] for r in rows]):.4f},"
            f"{np.mean([r['min_clearance'] for r in rows]):.4f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate an L6 PPO checkpoint.")
    parser.add_argument("--policy", required=True)
    parser.add_argument("--episodes-per-scenario", type=int, default=10)
    args = parser.parse_args()
    evaluate(Path(args.policy), args.episodes_per_scenario)


if __name__ == "__main__":
    main()
