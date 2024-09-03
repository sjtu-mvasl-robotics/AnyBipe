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


    def _reward_survival(self, idx):
        return 1.0 * self.env.dt[idx]

    

    def _reward_tracking_lin_vel(self, idx):
            env = self.env  # Do not skip this line. Afterwards, use env.{parameter_name} to access parameters of the environment.
            lin_vel_error = torch.sum(torch.square(env.commands[idx:idx+1,  :2] - env.base_lin_vel[idx:idx+1,  :2]), dim=1)
            tracking_sigma = 0.1  # Example value, adjust according to importance
            reward = torch.exp(-lin_vel_error / tracking_sigma)
            return 2.0 * reward

    def _reward_tracking_ang_vel(self, idx):
            env = self.env
            ang_vel_error = torch.square(env.commands[idx:idx+1,  2] - env.base_ang_vel[idx:idx+1,  2])
            tracking_sigma = 0.1  # Example value, adjust according to importance
            reward = torch.exp(-ang_vel_error / tracking_sigma)
            return 1.5 * reward

    def _reward_maintaining_base_height(self, idx):
            env = self.env
            desired_height = 0.65
            height_error = torch.abs(self._get_base_height() - desired_height)
            reward = torch.exp(-height_error / 0.1)  # Example value
            return 1.5 * reward

    def _reward_minimizing_action_rate(self, idx):
            env = self.env
            action_rate_penalty = torch.sum(torch.square(env.last_q[idx:idx+1] - env.q[idx:idx+1]), dim=1)
            return -0.5 * action_rate_penalty

    def _reward_ensuring_smooth_leg_movements(self, idx):
            env = self.env
            smooth_movement_penalty = torch.sum(torch.square(env.dq[idx:idx+1] - env.last_dq[idx:idx+1]), dim=1)
            return -0.5 * smooth_movement_penalty

    def _reward_penalizing_high_joint_torques(self, idx):
            env = self.env
            torque_limit_penalty = torch.sum((torch.abs(env.torques[idx:idx+1]) - env.torque_limits).clamp(min=0), dim=1)
            return -1.0 * torque_limit_penalty

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
                results[key] = value / total_timesteps
            
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
      
        

