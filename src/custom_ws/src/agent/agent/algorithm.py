import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions.categorical import Categorical

# h: 240
# w: 427
class RGBPolicy(nn.Module):
    def __init__(self, n_actions=10):
        super().__init__()
        self.backbone = nn.Sequential(
            nn.Conv2d(3, 32, 8, 4), nn.ReLU(),
            nn.Conv2d(32, 64, 4, 2), nn.ReLU(),
            nn.Conv2d(64, 64, 3, 1), nn.ReLU(),
            nn.Flatten(), 
            nn.Linear(64 * 7 * 7, 512), nn.ReLU(),
        )
        self.pi  = nn.Linear(512, n_actions)   # policy head
        self.vf  = nn.Linear(512, 1)           # value head

    def forward(self, x):
        z = self.backbone(x / 255.0)
        return self.pi(z), self.vf(z).squeeze(-1)

    # helper: returns action, log-prob, value
    def act(self, x):
        if x.ndim == 3: # For non-batched inputs...
            x = x.unsqueeze(0)  # (1, 3, 84, 84)
        logits, value = self(x)
        dist   = Categorical(logits=logits)
        action = dist.sample()
        return action, dist.log_prob(action), value
        
    def compute_returns(self, buffer):
        # turn the buffer into tensors
        obs_b, act_b, logp_b, val_b, rew_b, done_b = map(list, zip(*buffer))

        obs_b   = torch.stack(obs_b).float()   # (T,C,H,W)
        act_b   = torch.tensor(act_b)
        logp_b  = torch.tensor(logp_b)
        val_b   = torch.tensor(val_b)

        # GAE-lambda advantage
        returns = []
        adv     = []
        next_val = 0.0
        next_adv= 0.0
        gamma, lam = 0.99, 0.95
        for r, d, v in reversed(list(zip(rew_b, done_b, val_b))):
            delta   = r + gamma * (0 if d else next_val) - v
            next_adv = delta + gamma * lam * (0 if d else next_adv)
            adv.insert(0, next_adv)
            next_val = v
            returns.insert(0, next_adv + v)

        adv     = torch.tensor(adv)
        returns = torch.tensor(returns)

        # PPO update (one epoch, minibatch)
        clip_eps = 0.2
        optim = torch.optim.Adam(self.parameters(), lr=2.5e-4)

        for epoch in range(4):                          # 4 minibatch passes
            idx = torch.randperm(len(obs_b))
            for start in range(0, len(obs_b), 64):
                mb = idx[start:start+64]
                logits, value = self(obs_b[mb])
                dist = Categorical(logits=logits)

                ratio = torch.exp(dist.log_prob(act_b[mb]) - logp_b[mb])
                surr1 = ratio * adv[mb]
                surr2 = torch.clamp(ratio, 1-clip_eps, 1+clip_eps) * adv[mb]
                loss = -(torch.min(surr1, surr2)).mean()
                value_loss  = F.mse_loss(value.squeeze(-1), returns[mb])
                loss = loss + 0.5*value_loss - 0.01*dist.entropy().mean()

                optim.zero_grad()
                loss.backward()
                optim.step()
