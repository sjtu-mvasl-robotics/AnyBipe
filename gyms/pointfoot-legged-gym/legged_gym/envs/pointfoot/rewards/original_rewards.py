import torch
class OriginalRewards():
    def __init__(self, env):
        self.env = env

    def load_env(self, env):
        self.env = env

    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        return torch.square(self.env.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        # Penalize xy axes base angular velocity
        return torch.sum(torch.square(self.env.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self):
        # Penalize non flat base orientation
        return torch.sum(torch.square(self.env.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self):
        # Penalize base height away from target
        base_height = torch.mean(self.env.root_states[:, 2].unsqueeze(1) - self.env.measured_heights, dim=1)
        return torch.square(base_height - self.env.cfg.rewards.base_height_target)

    def _reward_torques(self):
        # Penalize torques
        return torch.sum(torch.square(self.env.torques), dim=1)

    def _reward_dof_vel(self):
        # Penalize dof velocities
        return torch.sum(torch.square(self.env.dof_vel), dim=1)

    def _reward_dof_acc(self):
        # Penalize dof accelerations
        return torch.sum(torch.square((self.env.last_dof_vel - self.env.dof_vel) / self.env.dt), dim=1)

    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.env.last_actions - self.env.actions), dim=1)

    def _reward_collision(self):
        # Penalize collisions on selected bodies
        return torch.sum(1. * (torch.norm(self.env.contact_forces[:, self.env.penalised_contact_indices, :], dim=-1) > 0.1),
                         dim=1)

    def _reward_termination(self):
        # Terminal reward / penalty
        return self.env.reset_buf * ~self.env.time_out_buf

    def _reward_dof_pos_limits(self):
        # Penalize dof positions too close to the limit
        out_of_limits = -(self.env.dof_pos - self.env.dof_pos_limits[:, 0]).clip(max=0.)  # lower limit
        out_of_limits += (self.env.dof_pos - self.env.dof_pos_limits[:, 1]).clip(min=0.)
        return torch.sum(out_of_limits, dim=1)

    def _reward_dof_vel_limits(self):
        # Penalize dof velocities too close to the limit
        # clip to max error = 1 rad/s per joint to avoid huge penalties
        return torch.sum(
            (torch.abs(self.env.dof_vel) - self.env.dof_vel_limits * self.env.cfg.rewards.soft_dof_vel_limit).clip(min=0., max=1.),
            dim=1)

    def _reward_torque_limits(self):
        # penalize torques too close to the limit
        return torch.sum(
            (torch.abs(self.env.torques) - self.env.torque_limits * self.env.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1)

    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.env.commands[:, :2] - self.env.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.env.cfg.rewards.tracking_sigma)

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.env.commands[:, 2] - self.env.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.env.cfg.rewards.tracking_sigma)

    def _reward_feet_air_time(self):
        # Reward steps between proper duration
        rew_airTime_below_min = torch.sum(
            torch.min(self.env.feet_air_time - self.env.cfg.rewards.min_feet_air_time,
                      torch.zeros_like(self.env.feet_air_time)) * self.env.first_contact,
            dim=1)
        rew_airTime_above_max = torch.sum(
            torch.min(self.env.cfg.rewards.max_feet_air_time - self.env.feet_air_time,
                      torch.zeros_like(self.env.feet_air_time)) * self.env.first_contact,
            dim=1)
        rew_airTime = rew_airTime_below_min + rew_airTime_above_max
        return rew_airTime

    def _reward_no_fly(self):
        contacts = self.env.contact_forces[:, self.env.feet_indices, 2] > 0.1
        single_contact = torch.sum(1. * contacts, dim=1) == 1
        return 1. * single_contact

    def _reward_unbalance_feet_air_time(self):
        return torch.var(self.env.last_feet_air_time, dim=-1)

    def _reward_unbalance_feet_height(self):
        return torch.var(self.env.last_max_feet_height, dim=-1)

    def _reward_stumble(self):
        # Penalize feet hitting vertical surfaces
        return torch.any(torch.norm(self.env.contact_forces[:, self.env.feet_indices, :2], dim=2) > \
                         5 * torch.abs(self.env.contact_forces[:, self.env.feet_indices, 2]), dim=1)

    def _reward_stand_still(self):
        # Penalize displacement and rotation at zero commands
        reward_lin = torch.abs(self.env.base_lin_vel[:, :2]) * (self.env.commands[:, :2] < 0.1)
        reward_ang = (torch.abs(self.env.base_ang_vel[:, -1]) * (self.env.commands[:, 2] < 0.1)).unsqueeze(dim=-1)
        return torch.sum(torch.cat((reward_lin, reward_ang), dim=-1), dim=-1)

    def _reward_feet_contact_forces(self):
        # penalize high contact forces
        return torch.sum((torch.norm(self.env.contact_forces[:, self.env.feet_indices, :],
                                     dim=-1) - self.env.cfg.rewards.max_contact_force).clip(min=0.), dim=1)

    def _reward_feet_distance(self):
        reward = 0
        for i in range(self.env.feet_state.shape[1] - 1):
            for j in range(i + 1, self.env.feet_state.shape[1]):
                feet_distance = torch.norm(
                    self.env.feet_state[:, i, :2] - self.env.feet_state[:, j, :2], dim=-1
                )
            reward += torch.clip(self.env.cfg.rewards.min_feet_distance - feet_distance, 0, 1)
        return reward

    def _reward_survival(self):
        return (~self.env.reset_buf).float() * self.env.dt

    def compute_success(self):
        lin_vel_error = self._reward_tracking_lin_vel()
        ang_vel_error = self._reward_tracking_ang_vel()
        # success if the reward is close to 1
        return lin_vel_error * ang_vel_error