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

# INSERT REMODULARIZED REWARD HERE

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
      
        

