import torch
import numpy as np
from legged_gym.utils.math import quat_apply_yaw, wrap_to_pi, torch_rand_sqrt_float, get_scale_shift
from isaacgym.torch_utils import *

class AnybipeRewards():
    def __init__(self, env):
        self.env = env

    def load_env(self, env):
        self.env = env

    def _tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.env.commands[:, :2] - self.env.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.env.cfg.rewards.tracking_sigma)

    def _tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.env.commands[:, 2] - self.env.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.env.cfg.rewards.tracking_sigma)
    
    def _survival(self):
        return (~self.env.reset_buf).float() * self.env.dt
    
    def _reward_gt_survival(self):
        return self._survival()
    
    def _get_base_height(self):
        return torch.mean(self.env.root_states[:, 2].unsqueeze(1) - self.env.measured_heights, dim=1)

# INSERT ANYBIPE REWARD HERE


    # Success criteria as the tracking of both linear and angular velocities
    def compute_success(self):
        lin_vel_error = self._tracking_lin_vel()
        ang_vel_error = self._tracking_ang_vel()
        reward_survival = ~self.env.reset_buf
        # success if the reward is close to 1
        return lin_vel_error * ang_vel_error * (reward_survival).float()
    
    def compute_reward(self):
        # return reward_names, reward_functions
        reward_names = []
        reward_functions = []
        for name, func in self.__class__.__dict__.items():
            if name.startswith('_reward_'):
                reward_functions.append(getattr(self, name))
                reward_names.append(name.replace("_reward_", ""))
        
        return reward_names, reward_functions

