import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

"""
An algorithm module must implement a Policy class.
A Policy class must implement act(), which returns what is consistent with the algorithm's trainer.
"""

import numpy as np




class Policy(nn.Module):
    def __init__(self, obs_shape=(3, 64, 64), act_dim=6):
        super().__init__()

    @torch.no_grad()
    def act(self, obs, deterministic=False):

