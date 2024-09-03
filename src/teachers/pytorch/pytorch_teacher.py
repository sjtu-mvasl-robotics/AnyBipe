from legged_gym import LEGGED_GYM_ROOT_DIR
import os

import isaacgym
import torch
import numpy as np
from legged_gym.envs import *
from legged_gym.utils import get_args, export_policy_as_jit, task_registry, Logger, get_load_path, class_to_dict
from rsl_rl.modules import ActorCritic, ActorCriticRecurrent
import numpy as np

file_path = __file__
file_root = os.path.dirname(file_path)
model_path = os.path.join(file_root, 'model.pt')
env_cfg, train_cfg = task_registry.get_cfgs(name="pointfoot_rough")
loaded_dict = torch.load(model_path)
if env_cfg.env.num_privileged_obs is None:
    env_cfg.env.num_privileged_obs = env_cfg.env.num_propriceptive_obs
actor_critic_class = eval(train_cfg.runner.policy_class_name)
actor_critic = actor_critic_class(env_cfg.env.num_propriceptive_obs, env_cfg.env.num_privileged_obs, env_cfg.env.num_actions, **class_to_dict(train_cfg.policy)
)
actor_critic.load_state_dict(loaded_dict['model_state_dict'])
model_ref = actor_critic.actor

def teacher(input_tensor: torch.Tensor) -> torch.Tensor:
    model = model_ref.to(input_tensor.device)
    output_tensor = model(input_tensor)
    return output_tensor
