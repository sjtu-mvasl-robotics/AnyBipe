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

    def _reward_tracking_lin_vel(self):
        env = self.env
        lin_vel_error = torch.sum(torch.square(env.commands[:, :2] - env.base_lin_vel[:, :2]), dim=1)
        reward = torch.exp(-lin_vel_error / env.cfg.rewards.tracking_sigma)
        return 1.0 * reward
    
    def _reward_tracking_ang_vel(self):
        env = self.env
        ang_vel_error = torch.square(env.commands[:, 2] - env.base_ang_vel[:, 2])
        reward = torch.exp(-ang_vel_error / env.cfg.rewards.tracking_sigma)
        return 1.0 * reward
    
    def _reward_maintain_base_height(self):
        env = self.env
        base_height_desired = 0.65
        base_height_error = torch.abs(self._get_base_height() - base_height_desired)
        reward = torch.exp(-base_height_error / env.cfg.rewards.height_sigma)
        return 1.0 * reward
    
    def _reward_minimize_action_rate(self):
        env = self.env
        action_rate = torch.sum(torch.square(env.last_actions - env.actions), dim=1)
        reward = -action_rate
        return 0.1 * reward
    
    def _reward_avoid_dof_limits(self):
        env = self.env
        dof_pos = env.dof_pos
        lower_limits = env.dof_pos_limits[:, 0]
        upper_limits = env.dof_pos_limits[:, 1]
        pos_error_lower = torch.clamp(lower_limits - dof_pos, min=0.0)
        pos_error_upper = torch.clamp(dof_pos - upper_limits, min=0.0)
        pos_error = pos_error_lower + pos_error_upper
        reward = -torch.sum(pos_error, dim=1)
        return 1.0 * reward
    
    def _reward_avoid_joint_collisions(self):
        env = self.env
        collision_force = torch.sum(1.0 * (torch.norm(env.contact_forces[:, env.penalised_contact_indices, :], dim=-1) > 0.1), dim=1)
        reward = -collision_force
        return 1.0 * reward
    
    def _reward_balanced_feet_movement(self):
        env = self.env
        feet_air_time_variance = torch.var(env.last_feet_air_time, dim=-1)
        reward = -feet_air_time_variance
        return 0.5 * reward
    
    def _reward_maintain_orientation(self):
        env = self.env
        gravity_vec = env.gravity_vec
        base_orientation = quat_apply(env.base_quat, gravity_vec)
        orientation_error = torch.abs(base_orientation[:, 2] + 1.0)  # Ensure perpendicular to gravity
        reward = torch.exp(-orientation_error / env.cfg.rewards.orientation_sigma)
        return 1.0 * reward
    
    def _reward_minimize_joint_velocities(self):
        env = self.env
        joint_velocities = torch.sum(torch.square(env.dof_vel), dim=1)
        reward = -joint_velocities
        return 0.1 * reward
    
    def _reward_minimize_joint_torques(self):
        env = self.env
        joint_torques = torch.sum(torch.square(env.torques), dim=1)
        reward = -joint_torques
        return 0.1 * reward


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


