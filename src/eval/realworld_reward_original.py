import numpy as np
import torch
from RewardCfg import *
import argparse
import warnings
warnings.filterwarnings("ignore")

class RealworldReward:
    def __init__(self, 
                 data_path, 
                 reward_type = "original"):
        
        self.data_path = data_path
        # load npz file
        self.env_old = np.load(data_path)
        class Env:
            pass
        self.env = Env()
        self.env_to_torch()
        self.reward_type = reward_type
        if reward_type == "original":
            self.reward = rewards_original()
        elif reward_type == "anybipe":
            self.reward = rewards_anybipe()
        else:
            raise ValueError("Invalid reward type")
        
    def env_to_torch(self):
        # convert each term of env to torch tensor
        for key in self.env_old.keys():
            # create attribute of env with name key
            setattr(self.env, key, torch.tensor(self.env_old[key]))
            self.env.dt = torch.diff(self.env.t)

    def _reward_lin_vel_z(self, idx):
            # Penalize z axis base linear velocity
            return torch.square(self.env.base_lin_vel[idx:idx+1,  2])

    def _reward_ang_vel_xy(self, idx):
            # Penalize xy axes base angular velocity
            return torch.sum(torch.square(self.env.base_ang_vel[idx:idx+1,  :2]), dim=1)

    def _reward_torques(self, idx):
            # Penalize torques
            return torch.sum(torch.square(self.env.torques[idx:idx+1]), dim=1)

    def _reward_dof_vel(self, idx):
            # Penalize dof velocities
            return torch.sum(torch.square(self.env.dq[idx:idx+1]), dim=1)


    def _reward_dof_acc(self, idx):
        return 1.0 * torch.sum(torch.square(self.env.ddq[idx:idx+1,:]), dim=1)
    

    def _reward_action_rate(self, idx):
            # Penalize changes in actions
            return torch.sum(torch.square(self.env.last_q[idx:idx+1] - self.env.q[idx:idx+1]), dim=1)

    def _reward_tracking_lin_vel(self, idx):
            # Tracking of linear velocity commands (xy axes)
            lin_vel_error = torch.sum(torch.square(self.env.commands[idx:idx+1,  :2] - self.env.base_lin_vel[idx:idx+1,  :2]), dim=1)
            return torch.exp(-lin_vel_error / self.reward.tracking_sigma)

    def _reward_tracking_ang_vel(self, idx):
            # Tracking of angular velocity commands (yaw)
            ang_vel_error = torch.square(self.env.commands[idx:idx+1,  2] - self.env.base_ang_vel[idx:idx+1,  2])
            return torch.exp(-ang_vel_error / self.reward.tracking_sigma)

    def _reward_stand_still(self, idx):
            # Penalize displacement and rotation at zero commands
            reward_lin = torch.abs(self.env.base_lin_vel[idx:idx+1,  :2]) * (self.env.commands[idx:idx+1,  :2] < 0.1)
            reward_ang = (torch.abs(self.env.base_ang_vel[idx:idx+1,  -1]) * (self.env.commands[idx:idx+1,  2] < 0.1)).unsqueeze(dim=-1)
            return torch.sum(torch.cat((reward_lin, reward_ang), dim=-1), dim=-1)

    def _reward_feet_distance(self, idx):
            reward = 0
            for i in range(self.env.feet_state.shape[1] - 1):
                for j in range(i + 1, self.env.feet_state.shape[1]):
                    feet_distance = torch.norm(
                        self.env.feet_state[idx:idx+1,  i, :2] - self.env.feet_state[idx:idx+1,  j, :2], dim=-1
                    )
                reward += torch.clip(self.reward.min_feet_distance - feet_distance, 0, 1)
            return reward


    def _reward_survival(self, idx):
        return 1.0 * self.env.dt[idx]

    

    def compute_reward(self):
            # return reward computation results
            results = {}
            reward_names = []
            reward_functions = []
            for name, func in self.__class__.__dict__.items():
                
                if name.startswith('_reward_'):
                    func_name = name.replace("_reward_", "")
                    reward_names.append(func_name)
                    reward_functions.append(getattr(self, name))
                    results[func_name] = 0.0 # initialize reward

            total_timesteps = self.env.survival_idx.numpy().item()
            if total_timesteps == -1:
                total_timesteps = self.env.t.shape[0]

            for i in range(total_timesteps):
                for j, func in enumerate(reward_functions):
                    try:
                        rew = (func(i)).numpy().item()
                        if self.reward_type == "original":
                             rew *= getattr(self.reward.scales, reward_names[j])
                        results[reward_names[j]] += rew
                        
                    except:
                        pass

            for key, value in results.items():
                results[key] = value / total_timesteps * self.env.t[-1].numpy().item()
            
            results["total_reward"] = np.sum(list(results.values()))
            results["total_survival_time"] = self.env.survival_time.numpy().item()
            
            return results
        
if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument("--data_path", type=str, default="exported_data/exported_data.npz")
    argparser.add_argument("--reward_type", type=str, default="original")
    args = argparser.parse_args()
    reward = RealworldReward(args.data_path, args.reward_type)
    results = reward.compute_reward()
    for key, value in results.items():
        # print value with precision 6
        print(key, ":{:.6f}".format(value))

    print('\n')
      
        

