# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym import LEGGED_GYM_ROOT_DIR
import os

import isaacgym
from legged_gym.envs import *
from legged_gym.utils import  get_args, export_policy_as_jit, task_registry, Logger

import numpy as np
import torch
from tqdm import tqdm

GLOBAL_DIR = os.getcwd()
GLOBAL_DIR = os.path.join(GLOBAL_DIR, '..')

def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    # override some parameters for testing
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 50)
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    # env_cfg.domain_rand.randomize_friction = False
    # env_cfg.domain_rand.push_robots = False
    if args.dr == "änybipe":
        print("Using Anybip domain randomization")
        env_cfg.domain_rand = env_cfg.domain_rand_anybipe

    # prepare environment
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()
    # load policy
    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)
    
    # export policy as a jit module (used to run it from C++)
    if EXPORT_POLICY:
        path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
        export_policy_as_jit(ppo_runner.alg.actor_critic, path)
        print('Exported policy as jit script to: ', path)

    logger = Logger(env.dt)
    robot_index = 0 # which robot is used for logging
    joint_index = 1 # which joint is used for logging
    stop_state_log = 100 # number of steps before plotting states
    stop_rew_log = env.max_episode_length + 1 # number of steps before print average episode rewards
    camera_position = np.array(env_cfg.viewer.pos, dtype=np.float64)
    camera_vel = np.array([1., 1., 0.])
    camera_direction = np.array(env_cfg.viewer.lookat) - np.array(env_cfg.viewer.pos)
    img_idx = 0

    for i in range(10*int(env.max_episode_length)):
        actions = policy(obs.detach())
        obs, _, rews, dones, infos = env.step(actions.detach())
        if RECORD_FRAMES:
            if i % 2:
                filename = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'frames', f"{img_idx}.png")
                env.gym.write_viewer_image_to_file(env.viewer, filename)
                img_idx += 1 
        if MOVE_CAMERA:
            camera_position += camera_vel * env.dt
            env.set_camera(camera_position, camera_position + camera_direction)

        if i < stop_state_log:
            logger.log_states(
                {
                    'dof_pos_target': actions[robot_index, joint_index].item() * env.cfg.control.action_scale,
                    'dof_pos': env.dof_pos[robot_index, joint_index].item(),
                    'dof_vel': env.dof_vel[robot_index, joint_index].item(),
                    'dof_torque': env.torques[robot_index, joint_index].item(),
                    'command_x': env.commands[robot_index, 0].item(),
                    'command_y': env.commands[robot_index, 1].item(),
                    'command_yaw': env.commands[robot_index, 2].item(),
                    'base_vel_x': env.base_lin_vel[robot_index, 0].item(),
                    'base_vel_y': env.base_lin_vel[robot_index, 1].item(),
                    'base_vel_z': env.base_lin_vel[robot_index, 2].item(),
                    'base_vel_yaw': env.base_ang_vel[robot_index, 2].item(),
                    'contact_forces_z': env.contact_forces[robot_index, env.feet_indices, 2].cpu().numpy()
                }
            )
        elif i==stop_state_log:
            logger.plot_states()
        if  0 < i < stop_rew_log:
            if infos["episode"]:
                num_episodes = torch.sum(env.reset_buf).item()
                if num_episodes>0:
                    logger.log_rewards(infos["episode"], num_episodes)
        elif i==stop_rew_log:
            logger.print_rewards()


def load_env(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    # override some parameters for testing
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 50)
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    # env_cfg.domain_rand.randomize_friction = False
    # env_cfg.domain_rand.push_robots = False
    if args.dr == "anybipe":
        print("Using Anybipe domain randomization")
        env_cfg.domain_rand = env_cfg.domain_rand_anybipe

    # prepare environment
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    # load policy
    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)

    

    
    # export policy as a jit module (used to run it from C++)
    if EXPORT_POLICY:
        path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
        export_policy_as_jit(ppo_runner.alg.actor_critic, path)
        print('Exported policy as jit script to: ', path)

    return env, policy

def play_dr(args):
    env, policy = load_env(args)
    obs = env.get_observations()
    num_eval_steps = args.eval_steps
    logger = Logger(env.dt)
    robot_index = 0 # which robot is used for logging
    # joint_index = 1 # which joint is used for logging

    if RECORD_FRAMES:
        import imageio
        mp4_writer = imageio.get_writer('pointfoot.mp4', fps=50)

    measured_x_vels = np.zeros(num_eval_steps)
    measured_y_vels = np.zeros(num_eval_steps)
    measured_z_vels = np.zeros(num_eval_steps)
    measured_yaw_vels = np.zeros(num_eval_steps)
    target_x_vels = np.zeros(num_eval_steps)
    target_y_vels = np.zeros(num_eval_steps)
    target_yaw_vels = np.zeros(num_eval_steps)
    joint_positions = np.zeros((num_eval_steps, 6))
    joint_velocities = np.zeros((num_eval_steps, 6))
    torques = np.zeros((num_eval_steps, 6))
    contact_forces = np.zeros((num_eval_steps, 2))
    base_height = np.zeros(num_eval_steps)

    starting_pos = env.root_states[0, :3].cpu().numpy()
    for i in tqdm(range(num_eval_steps)):
        actions = policy(obs.detach())
        obs, _, rews, dones, infos = env.step(actions.detach())
        measured_x_vels[i] = env.base_lin_vel[robot_index, 0].item()
        measured_y_vels[i] = env.base_lin_vel[robot_index, 1].item()
        measured_z_vels[i] = env.base_lin_vel[robot_index, 2].item()
        measured_yaw_vels[i] = env.base_ang_vel[robot_index, 2].item()

        target_x_vels[i] = env.commands[robot_index, 0].item()
        target_y_vels[i] = env.commands[robot_index, 1].item()
        target_yaw_vels[i] = env.commands[robot_index, 2].item()


        joint_positions[i] = env.dof_pos[robot_index].cpu().numpy()
        joint_velocities[i] = env.dof_vel[robot_index].cpu().numpy()
        torques[i] = env.torques[robot_index].cpu().numpy()

        contact_forces[i] = env.contact_forces[robot_index, env.feet_indices, 2].cpu().numpy()

        base_height[i] = env.root_states[robot_index, 2].item()

        if RECORD_FRAMES:
            temp_dir = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', 'play', 'exported', 'frames')
            if not os.path.exists(temp_dir):
                os.makedirs(temp_dir)
            tmp_path = os.path.join(temp_dir, f"{i}.png")
            
            env.gym.write_viewer_image_to_file(env.viewer, tmp_path)
            img = imageio.imread(tmp_path)
            mp4_writer.append_data(img)
        if infos["episode"]:
            num_episodes = torch.sum(env.reset_buf).item()
            if num_episodes>0:
                logger.log_rewards(infos["episode"], num_episodes)
    
    if RECORD_FRAMES:
        mp4_writer.close()
        video_dir_path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', 'play', 'exported', 'videos')
        if not os.path.exists(video_dir_path):
            os.makedirs(video_dir_path)
        import shutil
        shutil.move("pointfoot.mp4", os.path.join(video_dir_path, "play.mp4"))
    
    save_dir = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', 'play', 'exported')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    np.savez(os.path.join(save_dir, 'plot_data.npz'),
            measured_x_vels=measured_x_vels, measured_y_vels=measured_y_vels, measured_z_vels=measured_z_vels, measured_yaw_vels=measured_yaw_vels,
            target_x_vels=target_x_vels, target_y_vels=target_y_vels, target_yaw_vels=target_yaw_vels,
            joint_positions=joint_positions, joint_velocities=joint_velocities, torques=torques, contact_forces=contact_forces,
            base_height=base_height)
    
    print('Exported plot data to: ', os.path.join(save_dir, 'plot_data.npz'))
    
    # plot target and measured forward velocity
    from matplotlib import pyplot as plt
    # 8 subplots
    fig, axs = plt.subplots(2, 4, figsize=(20, 12))
    # x velocity and target x velocity
    axs[0, 0].plot(measured_x_vels, label='measured x velocity')
    axs[0, 0].plot(target_x_vels, label='target x velocity')
    axs[0, 0].set_title('X velocity')
    axs[0, 0].set_xlabel('steps')
    axs[0, 0].set_ylabel('velocity (m/s)')
    axs[0, 0].legend()
    # y velocity and target y velocity
    axs[0, 1].plot(measured_y_vels, label='measured y velocity')
    axs[0, 1].plot(target_y_vels, label='target y velocity')
    axs[0, 1].set_title('Y velocity')
    axs[0, 0].set_xlabel('steps')
    axs[0, 0].set_ylabel('velocity (m/s)')
    axs[0, 1].legend()
    # yaw velocity and target yaw velocity
    axs[0, 2].plot(measured_yaw_vels, label='measured yaw velocity')
    axs[0, 2].plot(target_yaw_vels, label='target yaw velocity')
    axs[0, 2].set_title('Yaw velocity')
    axs[0, 2].set_xlabel('steps')
    axs[0, 2].set_ylabel('velocity (rad/s)')
    axs[0, 2].legend()
    # measured z velocity
    axs[0, 3].plot(measured_z_vels, label='measured z velocity')
    axs[0, 3].set_title('Z velocity')
    axs[0, 3].set_xlabel('steps')
    axs[0, 3].set_ylabel('velocity (m/s)')
    axs[0, 3].legend()


    # joint positions
    for i in range(6):
        axs[1, 0].plot(joint_positions[:, i], label=f'joint {i}')
    axs[1, 0].set_title('Joint positions')
    axs[1, 0].set_xlabel('steps')
    axs[1, 0].set_ylabel('position (rad)')
    axs[1, 0].legend()

    # joint velocities
    for i in range(6):
        axs[1, 1].plot(joint_velocities[:, i], label=f'joint {i}')
    axs[1, 1].set_title('Joint velocities')
    axs[1, 1].set_xlabel('steps')
    axs[1, 1].set_ylabel('velocity (rad/s)')
    axs[1, 1].legend()

    # torques
    for i in range(6):
        axs[1, 2].plot(torques[:, i], label=f'joint {i}')
    axs[1, 2].set_title('Joint torques')
    axs[1, 2].set_xlabel('steps')
    axs[1, 2].set_ylabel('torque (Nm)')
    axs[1, 2].legend()

    # base height
    axs[1, 3].plot(base_height, label='base height')
    axs[1, 3].set_title('Base height')
    axs[1, 3].set_xlabel('steps')
    axs[1, 3].set_ylabel('height (m)')
    axs[1, 3].legend()

    if not args.headless:
        plt.show()

    # save plot
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'plot.png'))
    print('Exported plot to: ', os.path.join(save_dir, 'plot.png'))

    logger.print_rewards()



if __name__ == '__main__':
    EXPORT_POLICY = True
    RECORD_FRAMES = False
    MOVE_CAMERA = False
    args = get_args()
    play_dr(args)
    # play(args)
