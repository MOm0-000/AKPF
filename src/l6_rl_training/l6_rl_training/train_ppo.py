from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np
import torch
from torch import nn
from torch.distributions import Normal

from .env import L6AkpfEnv
from .geometry import SCENARIOS


class ActorCritic(nn.Module):
    def __init__(self, obs_size: int, action_size: int) -> None:
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_size, 96),
            nn.Tanh(),
            nn.Linear(96, 96),
            nn.Tanh(),
        )
        self.actor = nn.Linear(96, action_size)
        self.critic = nn.Linear(96, 1)
        self.log_std = nn.Parameter(torch.full((action_size,), -0.35))

    def forward(self, obs: torch.Tensor):
        hidden = self.shared(obs)
        mean = torch.tanh(self.actor(hidden))
        value = self.critic(hidden).squeeze(-1)
        return mean, value

    def act(self, obs: torch.Tensor):
        mean, value = self(obs)
        dist = Normal(mean, torch.exp(self.log_std))
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        return torch.clamp(action, -1.0, 1.0), log_prob, value

    def evaluate_actions(self, obs: torch.Tensor, actions: torch.Tensor):
        mean, value = self(obs)
        dist = Normal(mean, torch.exp(self.log_std))
        log_prob = dist.log_prob(actions).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return log_prob, entropy, value


@dataclass
class TrainConfig:
    obs_mode: str
    use_shield: bool
    updates: int
    num_envs: int
    rollout_steps: int
    seed: int
    lr: float
    bc_steps: int
    out: str


def make_envs(num_envs: int, obs_mode: str, use_shield: bool, seed: int) -> List[L6AkpfEnv]:
    names = list(SCENARIOS.keys())
    envs = []
    for i in range(num_envs):
        envs.append(L6AkpfEnv(names[i % len(names)], obs_mode, use_shield, seed + i))
    return envs


def collect_rollout(envs: List[L6AkpfEnv], model: ActorCritic, device: torch.device, rollout_steps: int):
    obs = np.stack([env._make_obs() for env in envs])
    buffers: Dict[str, List[np.ndarray]] = {k: [] for k in ["obs", "actions", "logp", "rewards", "dones", "values"]}
    episode_infos = []
    scenario_names = list(SCENARIOS.keys())

    for _ in range(rollout_steps):
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device)
        with torch.no_grad():
            action_t, logp_t, value_t = model.act(obs_t)
        actions = action_t.cpu().numpy()
        next_obs = []
        rewards = []
        dones = []
        for i, env in enumerate(envs):
            ob, reward, done, info = env.step(actions[i])
            if done:
                info = dict(info)
                info["scenario"] = env.scenario_name
                episode_infos.append(info)
                next_name = scenario_names[(scenario_names.index(env.scenario_name) + env.step_count + i) % len(scenario_names)]
                env.scenario_name = next_name
                env.scenario = SCENARIOS[next_name]
                ob = env.reset()
            next_obs.append(ob)
            rewards.append(reward)
            dones.append(done)

        buffers["obs"].append(obs)
        buffers["actions"].append(actions)
        buffers["logp"].append(logp_t.cpu().numpy())
        buffers["rewards"].append(np.asarray(rewards, dtype=np.float32))
        buffers["dones"].append(np.asarray(dones, dtype=np.float32))
        buffers["values"].append(value_t.cpu().numpy())
        obs = np.stack(next_obs)

    with torch.no_grad():
        last_values = model(torch.as_tensor(obs, dtype=torch.float32, device=device))[1].cpu().numpy()
    return buffers, last_values, episode_infos


def compute_gae(buffers, last_values, gamma=0.99, lam=0.95):
    rewards = np.asarray(buffers["rewards"], dtype=np.float32)
    dones = np.asarray(buffers["dones"], dtype=np.float32)
    values = np.asarray(buffers["values"], dtype=np.float32)
    adv = np.zeros_like(rewards)
    last_gae = np.zeros(rewards.shape[1], dtype=np.float32)
    for t in reversed(range(rewards.shape[0])):
        next_values = last_values if t == rewards.shape[0] - 1 else values[t + 1]
        nonterminal = 1.0 - dones[t]
        delta = rewards[t] + gamma * next_values * nonterminal - values[t]
        last_gae = delta + gamma * lam * nonterminal * last_gae
        adv[t] = last_gae
    returns = adv + values
    return adv, returns


def train(config: TrainConfig) -> Path:
    torch.manual_seed(config.seed)
    np.random.seed(config.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    envs = make_envs(config.num_envs, config.obs_mode, config.use_shield, config.seed)
    for env in envs:
        env.reset()
    model = ActorCritic(envs[0].observation_size, envs[0].action_size).to(device)
    optim = torch.optim.Adam(model.parameters(), lr=config.lr)

    if config.bc_steps > 0:
        bc_warm_start(envs, model, optim, device, config.bc_steps)

    out_dir = Path(config.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = out_dir / "metrics.csv"
    with metrics_path.open("w", encoding="utf-8") as f:
        f.write("update,episodes,success_rate,collision_rate,timeout_rate,mean_goal_dist,mean_min_clearance,mean_reward\n")

    last_mean_reward = 0.0
    for update in range(1, config.updates + 1):
        buffers, last_values, infos = collect_rollout(envs, model, device, config.rollout_steps)
        adv, returns = compute_gae(buffers, last_values)
        obs = torch.as_tensor(np.asarray(buffers["obs"]).reshape(-1, envs[0].observation_size), dtype=torch.float32, device=device)
        actions = torch.as_tensor(np.asarray(buffers["actions"]).reshape(-1, envs[0].action_size), dtype=torch.float32, device=device)
        old_logp = torch.as_tensor(np.asarray(buffers["logp"]).reshape(-1), dtype=torch.float32, device=device)
        adv_t = torch.as_tensor(adv.reshape(-1), dtype=torch.float32, device=device)
        returns_t = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=device)
        adv_t = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8)

        n = obs.shape[0]
        batch_size = min(256, n)
        idx = np.arange(n)
        for _ in range(4):
            np.random.shuffle(idx)
            for start in range(0, n, batch_size):
                mb = idx[start:start + batch_size]
                logp, entropy, value = model.evaluate_actions(obs[mb], actions[mb])
                ratio = torch.exp(logp - old_logp[mb])
                unclipped = ratio * adv_t[mb]
                clipped = torch.clamp(ratio, 0.80, 1.20) * adv_t[mb]
                policy_loss = -torch.min(unclipped, clipped).mean()
                value_loss = 0.5 * (returns_t[mb] - value).pow(2).mean()
                entropy_loss = -0.01 * entropy.mean()
                loss = policy_loss + value_loss + entropy_loss
                optim.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 0.5)
                optim.step()

        rewards_np = np.asarray(buffers["rewards"], dtype=np.float32)
        last_mean_reward = float(rewards_np.mean())
        if infos:
            success = np.mean([x["reached"] for x in infos])
            collision = np.mean([x["collision"] for x in infos])
            timeout = np.mean([x["timeout"] for x in infos])
            goal_dist = np.mean([x["goal_dist"] for x in infos])
            min_clearance = np.mean([x["min_clearance"] for x in infos])
        else:
            success = collision = timeout = goal_dist = min_clearance = 0.0

        with metrics_path.open("a", encoding="utf-8") as f:
            f.write(
                f"{update},{len(infos)},{success:.4f},{collision:.4f},{timeout:.4f},"
                f"{goal_dist:.4f},{min_clearance:.4f},{last_mean_reward:.4f}\n"
            )
        print(
            f"update={update:04d} episodes={len(infos):03d} success={success:.2f} "
            f"collision={collision:.2f} timeout={timeout:.2f} reward={last_mean_reward:.3f}",
            flush=True,
        )

    checkpoint = {
        "model_state": model.state_dict(),
        "config": asdict(config),
        "obs_size": envs[0].observation_size,
        "action_size": envs[0].action_size,
        "mean_reward": last_mean_reward,
    }
    ckpt_path = out_dir / "policy.pt"
    torch.save(checkpoint, ckpt_path)
    return ckpt_path


def parse_args() -> TrainConfig:
    parser = argparse.ArgumentParser(description="Train a lightweight PPO policy in the L6 simplified AKPF env.")
    parser.add_argument("--obs-mode", choices=["baseline", "akpf", "full"], default="akpf")
    parser.add_argument("--use-shield", action="store_true")
    parser.add_argument("--updates", type=int, default=40)
    parser.add_argument("--num-envs", type=int, default=10)
    parser.add_argument("--rollout-steps", type=int, default=128)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--bc-steps", type=int, default=200)
    parser.add_argument("--out", default="/tmp/l6_ppo_akpf")
    args = parser.parse_args()
    return TrainConfig(**vars(args))


def bc_warm_start(
    envs: List[L6AkpfEnv],
    model: ActorCritic,
    optim: torch.optim.Optimizer,
    device: torch.device,
    bc_steps: int,
) -> None:
    old_lrs = [group["lr"] for group in optim.param_groups]
    for group in optim.param_groups:
        group["lr"] = max(group["lr"], 1e-3)
    last_loss = 0.0
    for env in envs:
        env.reset()
    obs_rows = []
    action_rows = []
    for step in range(max(1, bc_steps)):
        for env in envs:
            obs_rows.append(env._make_obs())
            action = env.teacher_action()
            action_rows.append(action)
            obs, _, done, _ = env.step(action)
            if done:
                env.reset()
        if len(obs_rows) >= 512 or step == bc_steps - 1:
            obs_t = torch.as_tensor(np.asarray(obs_rows), dtype=torch.float32, device=device)
            action_t = torch.as_tensor(np.asarray(action_rows), dtype=torch.float32, device=device)
            for _ in range(6):
                mean, _ = model(obs_t)
                loss = torch.mean((mean - action_t) ** 2)
                optim.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 0.5)
                optim.step()
                last_loss = float(loss.detach().cpu())
            obs_rows.clear()
            action_rows.clear()
    for group, lr in zip(optim.param_groups, old_lrs):
        group["lr"] = lr
    print(f"bc_warm_start_steps={bc_steps} final_mse={last_loss:.6f}", flush=True)


def main() -> None:
    ckpt_path = train(parse_args())
    print(f"saved_policy={ckpt_path}")


if __name__ == "__main__":
    main()
