import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions.categorical import Categorical

"""
An algorithm module must implement a Policy class.
A Policy class must implement act(), which returns what is consistent with the algorithm's trainer.
"""

import numpy as np

# Simple MLP helper
def mlp(input_dim, output_dim, hidden_sizes=(256, 256), activation=nn.ReLU, output_activation=None):
    layers = []
    prev_dim = input_dim
    for h in hidden_sizes:
        layers += [nn.Linear(prev_dim, h), activation()]
        prev_dim = h
    layers.append(nn.Linear(prev_dim, output_dim))
    if output_activation is not None:
        layers.append(output_activation())
    return nn.Sequential(*layers)

class GaussianPolicy(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden_sizes=(256, 256), log_std_bounds=(-20, 2)):
        super().__init__()
        self.net = mlp(obs_dim, act_dim * 2, hidden_sizes)
        self.log_std_bounds = log_std_bounds

    def forward(self, obs):
        mu_log_std = self.net(obs)
        mu, log_std = torch.chunk(mu_log_std, 2, dim=-1)
        log_std = torch.tanh(log_std)
        log_std_min, log_std_max = self.log_std_bounds
        log_std = log_std_min + 0.5 * (log_std + 1.0) * (log_std_max - log_std_min)
        std = torch.exp(log_std)
        return mu, std

    def sample(self, obs):
        mu, std = self.forward(obs)
        dist = torch.distributions.Normal(mu, std)
        x_t = dist.rsample()
        y_t = torch.tanh(x_t)
        log_prob = dist.log_prob(x_t).sum(dim=-1, keepdim=True)
        log_prob -= torch.log(1 - y_t.pow(2) + 1e-6).sum(dim=-1, keepdim=True)
        return y_t, log_prob

class QNetwork(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden_sizes=(256, 256)):
        super().__init__()
        self.net = mlp(obs_dim + act_dim, 1, hidden_sizes)

    def forward(self, obs, act):
        return self.net(torch.cat([obs, act], dim=-1))

class Policy(nn.Module):
    def __init__(self, obs_dim, act_dim=6):
        super().__init__()
        self.actor = GaussianPolicy(obs_dim, act_dim)
        self.q1 = QNetwork(obs_dim, act_dim)
        self.q2 = QNetwork(obs_dim, act_dim)
        self.target_q1 = QNetwork(obs_dim, act_dim)
        self.target_q2 = QNetwork(obs_dim, act_dim)

        # Entropy temperature
        self.log_alpha = nn.Parameter(torch.zeros(1))
        self.alpha = self.log_alpha.exp()

    @torch.no_grad()
    def act(self, obs, deterministic=False):
        obs = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
        if deterministic:
            mu, _ = self.actor.forward(obs)
            action = torch.tanh(mu)
        else:
            action, _ = self.actor.sample(obs)
        return action.cpu().numpy()[0]
