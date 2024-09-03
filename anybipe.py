'''
    The implementation of this script partly refers to the Eureka project by NVIDIA.
    The original implementation can be found at https://github.com/eureka-research/Eureka.
    This is not a distributed version of Eureka.
'''
import hydra
import numpy as np 
import json
import logging 
import matplotlib.pyplot as plt
import os
import openai
import re
import subprocess
from pathlib import Path
import shutil
import time 

from utils.misc import * 
from utils.extract_task_code import *

ROOT_DIR = os.getcwd()

@hydra.main(config_path='cfg', config_name='config', version_base='1.1')
def main(cfg):

    # Initialize gpu configs
    multi_gpu = cfg.cuda.multi_gpu
    if multi_gpu:
        gpu_indices = cfg.cuda.visible_devices
        if len(gpu_indices) == 0:
            logging.error("Please specify the GPU indices for multi-GPU training!")
            exit()

        logging.info(f"Using multi-GPU for training: {gpu_indices}")
        logging.info(f"Using CUDA devices: {gpu_indices}")
        os.environ["CUDA_VISIBLE_DEVICES"] = gpu_indices
        print("CUDA_VISIBLE_DEVICES:", os.environ["CUDA_VISIBLE_DEVICES"])
    else:
        gpu_indices = [0]
        logging.info("Using single GPU for training!")

    if cfg.cuda.deterministic:
        # Set the random seed for reproducibility
        seed = cfg.cuda.seed
        logging.info(f"Using random seed: {seed}")
        np.random.seed(seed)

    # Set up workspace configurations
    workspace_dir = cfg.workspace_dir
    workspace_dir = os.path.join(ROOT_DIR, 'gyms', workspace_dir)
    logging.info(f"Workspace: {workspace_dir}")
    logging.info(f"Project Root: {ROOT_DIR}")
    if not os.path.exists(workspace_dir):
        logging.error(f"Workspace directory {workspace_dir} does not exist!")
        exit()

    logging.info(f"Using OpenAI API Key: {cfg.api.api_key}")
    logging.info(f"Using OpenAI API Base: {cfg.api.api_base}")

    # setup cache directory
    cache_dir = os.path.join(ROOT_DIR, 'cache')
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
        logging.info(f"Cache directory created at {cache_dir}")

    # Set up OpenAI API
    openai.api_key = cfg.api.api_key
    openai.api_base = cfg.api.api_base

    model = cfg.model
    logging.info(f"Training AnyBipe with LLM model: {model}")

    # Set up the task configurations
    task_name = cfg.env.task
    env_name = cfg.env.env_name
    description = cfg.env.description
    custom_obs_list = cfg.env.custom_obs
    custom_obs_list = custom_obs_list.split(",") if custom_obs_list else []
    custom_obs_list = [obs.strip() for obs in custom_obs_list]
    custom_obs_weights = cfg.env.custom_obs_weights
    custom_obs_weights = custom_obs_weights.split(",") if custom_obs_weights else []
    custom_obs_weights = [float(weight) for weight in custom_obs_weights]
    assert len(custom_obs_list) == len(custom_obs_weights), "Length of custom observation list and weights must be the same!"
    success_weight = cfg.env.success_weight
    correlation_weight = cfg.env.correlation_weight
    logging.info(f"Custom Observations: {custom_obs_list}")
    logging.info(f"Custom Observation Weights: {custom_obs_weights}")
    logging.info(f"Success Weight: {success_weight}")
    logging.info(f"Correlation Weight: {correlation_weight}")

    logging.info(f"Task: {task_name}")
    logging.info(f"Environment: {env_name}")
    logging.info(f"Description: {description}")

    # Initialize resource files

    reward_template = os.path.join(workspace_dir, cfg.env.reward_template)
    reward_template_str = file_to_string(reward_template)
    realworld_reward_template = os.path.join(ROOT_DIR, 'src', 'eval', 'realworld_reward_template.py')
    reward_dest = os.path.join(workspace_dir, cfg.env.reward_dest)
    obs_file = os.path.join(ROOT_DIR, 'envs', f'{env_name}.py')
    obs_file_str = file_to_string(obs_file)
    shutil.copy(obs_file, f'env_{env_name}_obs.py')
    config_template_dir = os.path.join(workspace_dir, cfg.env.config_template_dir)
    config_template = file_to_string(config_template_dir)
    config_dest = os.path.join(workspace_dir, cfg.env.config_dir)
    if cfg.env.use_teacher:
        config_template = re.sub(r'use_teacher_model = False', 'use_teacher_model = True', config_template)
        teacher_model_path = os.path.join(ROOT_DIR, cfg.env.teacher_dir)
        config_template = re.sub(r'teacher_function_path = None', f"teacher_function_path = '{teacher_model_path}'", config_template)
        config_template = re.sub(r'teacher_coef = 0.0', f"teacher_coef = {cfg.env.teacher_coef}", config_template)

    with open(config_dest, 'w') as f:
        f.writelines(config_template + "\n")

    # Load text prompts
    prompts_dir = os.path.join(ROOT_DIR, 'prompts')
    prompts_general = os.path.join(prompts_dir, 'general')
    prompts_env = os.path.join(prompts_dir, 'env', env_name)

    # Load general prompts
    initial_system = load_prompts(prompts_general, 'initial_system')
    coding_tips = load_prompts(prompts_general, 'coding_tips')
    coding_restrictions = load_prompts(prompts_general, 'coding_restrictions')
    initial_user = load_prompts(prompts_general, 'initial_user')
    safety_restrictions = load_prompts(prompts_general, 'safety_restrictions')
    code_feedback = load_prompts(prompts_general, 'code_feedback')
    execution_error_feedback = load_prompts(prompts_general, 'execution_error_feedback')
    policy_feedback = load_prompts(prompts_general, 'policy_feedback')
    ros_feedback = load_prompts(prompts_general, 'ros_feedback')
    real_feedback = load_prompts(prompts_general, 'real_feedback')

    # Load environment-specific prompts
    reward_sign = load_prompts(prompts_env, 'reward_sign')
    coding_references = load_prompts(prompts_env, 'coding_references') if cfg.env.prompts.additional.coding_references else ''
    if cfg.env.prompts.additional.coding_tips:
        coding_tips += load_prompts(prompts_env, 'coding_tips')
    if cfg.env.prompts.additional.coding_restrictions:
        coding_restrictions += load_prompts(prompts_env, 'coding_restrictions')
    if cfg.env.prompts.additional.safety_restrictions:
        safety_restrictions += load_prompts(prompts_env, 'safety_restrictions')
    if cfg.env.prompts.additional.policy_feedback:
        policy_feedback += load_prompts(prompts_env, 'policy_feedback')
    if cfg.env.prompts.additional.ros_feedback:
        ros_feedback += load_prompts(prompts_env, 'ros_feedback')
    if cfg.env.prompts.additional.real_feedback:
        real_feedback += load_prompts(prompts_env, 'real_feedback')

    initial_system = (initial_system.format(reward_sign=reward_sign) 
                      + coding_tips 
                      + coding_restrictions 
                      + coding_references 
                      + safety_restrictions)
    initial_user = initial_user.format(obs_file_str=obs_file_str, description=description)
    messages = [{"role": "system", "content": initial_system}, {"role": "user", "content": initial_user}]
    
    DUMMY_FAILURE = -1e5
    max_successes = []
    max_successes_reward_correlation = []
    execute_rates = []
    best_code_paths = []
    best_ckpt_paths = []
    max_success_overall = DUMMY_FAILURE
    max_success_reward_correlation_overall = DUMMY_FAILURE
    max_reward_code_path = None 

    # Extract ground truth reward
    if cfg.gt.use_gt:
        if not cfg.gt.set_gt:
            logging.info("Ground-truth reward not set, training ground-truth reward model...")
            rl_filepath = f"env_gt_response.txt"
            with open(rl_filepath, 'w') as f:
                command = f"python3 -u {workspace_dir}/{cfg.env.train_script} --task={env_name} --reward=original --max_iterations={cfg.env.train_iterations} --headless"
                command = command.split(" ")
                process = subprocess.Popen(command, stdout=f, stderr=f)
                logging.info("Training ground-truth reward model...")
            block_until_training(rl_filepath, success_keyword=cfg.env.success_keyword, failure_keyword=cfg.env.failure_keyword, log_status=True)
        
        else:
            rl_filepath = cfg.gt.gt_dir
            logging.info(f"Ground-truth reward set, using {rl_filepath} as ground-truth reward!")
        
        run_log = construct_run_log_gt(file_to_string(rl_filepath))
        gt_reward = run_log["gt_reward"]
        gt_log = run_log
        logging.info(f"Ground-truth reward extracted successfully!")
    else:
        gt_reward = None
        gt_log = None
        logging.info("Ground-truth reward not used!")

    # main loop
    for iter in range(cfg.iteration):
        # Generation part: refering to Eureka algorithm
        # Original implementation can be found at https://github.com/eureka-research/Eureka
        responses = []
        response_cur = None
        total_samples = 0
        total_token = 0
        total_completion_token = 0
        chunk_size = cfg.sample if "gpt-3.5" in model else 2

        logging.info(f"Iteration {iter}: Generating {cfg.sample} samples with {cfg.model}")

        while True:
            if total_samples >= cfg.sample:
                break
            for attempt in range(5):
                try:
                    response_cur = openai.ChatCompletion.create(
                        model=model,
                        messages=messages,
                        temperature=cfg.temperature,
                        n=chunk_size,
                    )
                    total_samples += chunk_size
                    break
                except Exception as e:
                    if attempt >= 10:
                        chunk_size = max(int(chunk_size / 2), 1)
                        print("Current Chunk Size", chunk_size)
                    logging.info(f"Attempt {attempt+1} failed with error: {e}")
                    time.sleep(1)
            if response_cur is None:
                logging.info("Code terminated due to too many failed attempts!")
                exit()

            responses.extend(response_cur["choices"])
            prompt_tokens = response_cur["usage"]["prompt_tokens"]
            total_completion_token += response_cur["usage"]["completion_tokens"]
            total_token += response_cur["usage"]["total_tokens"]

            print(response_cur["choices"][0]["message"]["content"])

        if cfg.sample == 1:
            logging.info(f"Iteration {iter}: GPT Output:\n " + responses[0]["message"]["content"] + "\n")

        # Test if responses are fully generated
        if len(responses) < cfg.sample:
            logging.info(f"Iteration {iter}: Not enough responses generated, retriving {len(responses)} responses, retrying code generation. Chunk Size: {chunk_size}, total sample: {total_samples}, continue training with this configuration!")

        else:
            logging.info(f"Generated {len(responses)} responses successfully!")

        # Logging Token Information
        logging.info(f"Iteration {iter}: Prompt Tokens: {prompt_tokens}, Completion Tokens: {total_completion_token}, Total Tokens: {total_token}")

        code_runs = []
        rl_runs = []
        for response_id in range(min(cfg.sample, len(responses))):
            response_cur = responses[response_id]["message"]["content"]
            logging.info(f"Iteration {iter}: Processing Code Run {response_id}")

            # Regex patterns to extract python code enclosed in GPT response
            patterns = [
                r'```python(.*?)```',
                r'```(.*?)```',
                r'"""(.*?)"""',
                r'""(.*?)""',
                r'"(.*?)"',
            ]
            for pattern in patterns:
                code_string = re.search(pattern, response_cur, re.DOTALL)
                if code_string is not None:
                    code_string = code_string.group(1).strip()
                    break
            code_string = response_cur if not code_string else code_string

            # Remove unnecessary imports
            lines = code_string.split("\n")
            lines = [" "*4 + line for line in lines]
            for i, line in enumerate(lines):
                if line.strip().startswith("def "):
                    code_string = "\n".join(lines[i:])
                    break
            
            code_runs.append(code_string)

            cur_reward_str = reward_template_str.replace("# INSERT ANYBIPE REWARD HERE", code_string)
            with open(reward_dest, 'w') as f:
                f.writelines(cur_reward_str + "\n")

            with open(f"env_{env_name}_iter{iter}_response{response_id}_reward.py", 'w') as f:
                f.writelines(code_string + "\n")

            shutil.copy(reward_dest, f"env_{env_name}_iter{iter}_response{response_id}.py")

            # execute training script
            rl_filepath = f"env_{env_name}_iter{iter}_response{response_id}.txt"
            with open(rl_filepath, 'w') as f:
                command = f"python3 -u {workspace_dir}/{cfg.env.train_script} --task={env_name} --reward=anybipe --max_iterations={cfg.env.train_iterations} --headless"
                command = command.split(" ")
                process = subprocess.Popen(command, stdout=f, stderr=f)
            block_until_training(rl_filepath, success_keyword=cfg.env.success_keyword, failure_keyword=cfg.env.failure_keyword,
                                 log_status=True, iter_num=iter, response_id=response_id)
            rl_runs.append(process)

        code_feedbacks = []
        class CustomObs:
            pass
        custom_obs = CustomObs()
        contents = []
        successes = []
        reward_correlations = []
        code_paths = []
        ckpt_paths = []

        exec_success = False
        for response_id, (code_run, rl_run) in enumerate(zip(code_runs, rl_runs)):
            rl_run.communicate()
            rl_filepath = f"env_{env_name}_iter{iter}_response{response_id}.txt"
            code_paths.append(f"env_{env_name}_iter{iter}_response{response_id}.py")
            try:
                with open(rl_filepath, 'r') as f:
                    stdout_str = f.read()
            except:
                content = execution_error_feedback.format(traceback_msg="Code Run cannot be executed due to function signature error! Please re-write an entirely new reward function!")
                content += coding_tips
                content += coding_restrictions
                contents.append(content)
                successes.append(DUMMY_FAILURE)
                reward_correlations.append(DUMMY_FAILURE)
                ckpt_paths.append(None)
                continue

            content = ''
            traceback_msg = filter_traceback(stdout_str)
            if len(traceback_msg) == 0: # No error
                exec_success = True
                run_log = construct_run_log(stdout_str)
                train_iterations = np.array(run_log['iterations']).shape[0]
                epoch_freq = max(int(train_iterations // 10), 1)
                epochs_per_log = 10
                content += policy_feedback.format(epoch_freq=epochs_per_log*epoch_freq)

                # Compute correlation between ground-truth reward (human engineered) and the reward function generated by GPT
                if gt_reward is not None:
                    run_log["gt_reward"] = gt_reward

                else:
                    run_log["gt_reward"] = run_log["gpt_reward"]
                    logging.info(f"Iteration {iter}: Ground-truth reward not used, using GPT reward (first time) as ground-truth reward!")

                gt_reward = run_log["gt_reward"]
                gpt_reward = run_log["gpt_reward"]
                reward_correlation = np.corrcoef(gt_reward, gpt_reward)[0, 1]
                reward_correlations.append(reward_correlation)
                ckpt_paths.append(run_log["save_path"])
                #logging.info("Iter {} Response {}: \n \tGPT Reward Final: {} \n \tCorrelation: {} \n \tCheckpoint Path: {}".format(iter, response_id, gpt_reward[-1], reward_correlation, run_log["save_path"]))

                for metric in sorted(run_log.keys()):
                    if metric == "save_path":
                        continue
                    if "timesteps" in metric:
                        continue

                    metric_cur = ['{:.2f}'.format(x) for x in run_log[metric][::epoch_freq]
                                    ]
                    metric_cur_max = max(run_log[metric])
                    metric_cur_mean = sum(run_log[metric]) / len(run_log[metric])
                    metric_cur_min = min(run_log[metric])
                    for c_obs in custom_obs_list:
                        if c_obs in metric:
                            try:
                                getattr(custom_obs, c_obs).append(run_log[metric][-1])
                            except: # Initialize the list
                                setattr(custom_obs, c_obs, [run_log[metric][-1]])

                    if "rew_success" == metric:
                        successes.append(metric_cur_max)

                    if metric != "gt_reward" and metric != "gpt_reward":
                        if metric != "rew_success":
                            metric_name = metric 
                        else:
                            metric_name = "task score"
                        content += f"{metric_name}: {metric_cur}, Max: {metric_cur_max:.2f}, Mean: {metric_cur_mean:.2f}, Min: {metric_cur_min:.2f} \n"                    
                    else:
                        # Provide ground-truth score when success rate not applicable
                        if "rew_success" not in run_log:
                            content += f"ground-truth score: {metric_cur}, Max: {metric_cur_max:.2f}, Mean: {metric_cur_mean:.2f}, Min: {metric_cur_min:.2f} \n"             
                code_feedbacks.append(code_feedback)
                content += code_feedback

            else:
                # Provide traceback message when code execution fails
                successes.append(DUMMY_FAILURE)
                reward_correlations.append(DUMMY_FAILURE)
                ckpt_paths.append(None)
                for c_obs in custom_obs_list:
                    try:
                        getattr(custom_obs, c_obs).append(DUMMY_FAILURE)
                    except:
                        setattr(custom_obs, c_obs, [DUMMY_FAILURE])
                content = execution_error_feedback.format(traceback_msg=traceback_msg)

            content += coding_tips
            content += coding_restrictions
            contents.append(content)

        # Repeat the iteration if all code generation failed
        if not exec_success and cfg.sample != 1:
            execute_rates.append(0.)
            max_successes.append(DUMMY_FAILURE)
            max_successes_reward_correlation.append(DUMMY_FAILURE)
            best_code_paths.append(None)
            best_ckpt_paths.append(None)
            logging.info("All code generation failed! Repeat this iteration from the current message checkpoint!")
            continue

        # Select the top 15% code sample to evaluate in simulation (gazebo)
        top_num = int(np.ceil(0.15 * len(code_runs)))
        weighted_sum = success_weight * np.array(successes) + correlation_weight * np.array(reward_correlations)
        for idx, c_obs in enumerate(custom_obs_list):
            try:
                arr = np.array(getattr(custom_obs, c_obs))
                if len(arr) == len(successes):
                    weighted_sum += custom_obs_weights[idx] * arr
            except:
                pass

        top_idx = np.argsort(weighted_sum)[-top_num:]

        # examine and remove the ones with negative weighted_sum from the top_idx
        top_idx = [idx for idx in top_idx if weighted_sum[idx] >= 0]
        logging.info(f"Iteration {iter}: Top {top_num} code samples selected for evaluation: {top_idx}")

        if len(top_idx) == 0:
            logging.info(f"Iteration {iter}: No positive weighted sum found, skipping evaluation!")
            continue

        # Evaluate the top code samples in simulation
        deployment_feedbacks = []
        deployment_original_feedbacks = []
        deployment_rewards = []
        deployment_survival_times = []

        for i, idx in enumerate(top_idx):
            code_path = code_paths[idx]
            ckpt_path = ckpt_paths[idx]
            data_path = model_deployment_test(
                model_path=ckpt_path,
                env_name=env_name,
                gym_path=workspace_dir,
                deploy_path=os.path.join(ROOT_DIR, cfg.deploy_dir),
                script_path=os.path.join(ROOT_DIR, "src", "eval"),
                monitor_py="eval.py",
                tracking_duration=cfg.env.tracking_duration,
                log_dir=cache_dir,
                ROOT_DIR=ROOT_DIR
            )
            if data_path is None:
                logging.info(f"Iteration {iter}: Evaluation failed for code path: {code_path}")
                continue
            func_map_dir = os.path.join(ROOT_DIR, cfg.deploy_gazebo_map)
            template_file_path = os.path.join(ROOT_DIR, 'src', 'eval', 'realworld_reward_template.py')

            conv_failed = convert_reward_file(code_path, template_file_path, import_module_from_file(script_path=func_map_dir, function_name="func_map"))
            if len(conv_failed) > 0:
                logging.info(f"Iteration {iter}: Conversion failed for these functions:\n {conv_failed}")

            command = f"python3 -u {ROOT_DIR}/src/eval/realworld_reward.py --data_path={data_path} --reward_type=anybipe"
            command = command.split(" ")
            deploy_eval_filepath = f"env_{env_name}_iter{iter}_response{idx}_deploy_eval.txt"
            with open(deploy_eval_filepath, 'w') as f:
                process = subprocess.Popen(command, stdout=f, stderr=f)
                process.wait()

            deploy_str = file_to_string(deploy_eval_filepath)
            if "total_reward" not in deploy_str:
                deployment_feedbacks.append(f'Evaluation failed for iteration {iter} response {idx}, no feedback available!')
                deployment_rewards.append(DUMMY_FAILURE)
                deployment_survival_times.append(DUMMY_FAILURE)

            else:
                deployment_feedbacks.append(f'Function convertion failed for the following functions:\n{conv_failed}\n Running evaluation for iteration {iter} response {idx}:\n {deploy_str}')
                lines = deploy_str.split("\n")
                for line in lines:
                    if "total_reward" in line:
                        deployment_rewards.append(float(line.split(":")[1].strip()))
                        break

                for line in lines:
                    if "total_survival_time" in line:
                        deployment_survival_times.append(float(line.split(":")[1].strip()))
                        break

                logging.info(f"Iteration {iter}: Evaluation for code path {code_path} completed successfully!")

            command = f"python3 -u {ROOT_DIR}/src/eval/realworld_reward_original.py --data_path={data_path}"
            command = command.split(" ")
            deploy_eval_original_filepath = f"env_{env_name}_iter{iter}_response{idx}_deploy_eval_original.txt"
            with open(deploy_eval_original_filepath, 'w') as f:
                process = subprocess.Popen(command, stdout=f, stderr=f)
                process.wait()

            deploy_original_str = file_to_string(deploy_eval_original_filepath)
            deployment_original_feedbacks.append(f'Human engineered reward function evaluation for iteration {iter} response {idx}:\n {deploy_original_str}')

            contents[idx] += ros_feedback.format(sim_time=cfg.env.tracking_duration)
            contents[idx] += deployment_feedbacks[i]
            contents[idx] += deployment_original_feedbacks[i]


        # Evaluate the top code samples in real-world
        realworld_feedbacks = []
        realworld_original_feedbacks = []
        realworld_rewards = []
        realworld_survival_times = []

        if cfg.test_realworld:
            for i, idx in enumerate(top_idx):
                # Safety check: if deployment survival time is less than 0.9 * tracking_duration, skip real-world evaluation
                if deployment_survival_times[idx] < 0.9 * cfg.env.tracking_duration:
                    logging.info(f"Iteration {iter}: Deployment survival time is less than 90% of tracking duration, skipping real-world evaluation!")
                    realworld_feedbacks.append(f'Deployment survival time is less than 90% of tracking duration, skipping real-world evaluation!')
                    realworld_original_feedbacks.append(f'Deployment survival time is less than 90% of tracking duration, skipping real-world evaluation!')
                    realworld_rewards.append(DUMMY_FAILURE)
                    realworld_survival_times.append(DUMMY_FAILURE)
                    continue


                code_path = code_paths[idx]
                ckpt_path = ckpt_paths[idx]
                data_path = model_deployment_test(
                    model_path=ckpt_path,
                    env_name=env_name,
                    gym_path=workspace_dir,
                    deploy_path=os.path.join(ROOT_DIR, cfg.deploy_dir),
                    script_path=os.path.join(ROOT_DIR, "src", "eval"),
                    monitor_py="eval_real.py",
                    tracking_duration=cfg.env.tracking_duration,
                    log_dir=cache_dir,
                    ROOT_DIR=ROOT_DIR
                )
                if data_path is None:
                    logging.info(f"Iteration {iter}: Real-world evaluation failed for code path: {code_path}")
                    continue
                func_map_dir = os.path.join(ROOT_DIR, cfg.deploy_realworld_map)
                template_file_path = os.path.join(ROOT_DIR, 'src', 'eval', 'realworld_reward_template.py')

                conv_failed = convert_reward_file(code_path, import_module_from_file(script_path=func_map_dir, template_file_path = template_file_path, function_name="func_map"))
                if len(conv_failed) > 0:
                    logging.info(f"Iteration {iter}: Conversion failed for these functions:\n {conv_failed}")

                command = f"python3 -u {ROOT_DIR}/src/eval/realworld_reward.py --data_path={data_path} --reward_type=anybipe"
                command = command.split(" ")
                real_eval_filepath = f"env_{env_name}_iter{iter}_response{idx}_real_eval.txt"
                with open(real_eval_filepath, 'w') as f:
                    process = subprocess.Popen(command, stdout=f, stderr=f)
                    process.wait()

                real_str = file_to_string(real_eval_filepath)
                if "total_reward" not in real_str:
                    realworld_feedbacks.append(f'Realworld evaluation failed for iteration {iter} response {idx}, no feedback available!')
                    realworld_rewards.append(DUMMY_FAILURE)
                    realworld_survival_times.append(DUMMY_FAILURE)

                else:
                    realworld_feedbacks.append(f'Function convertion failed for the following functions:\n{conv_failed}\n Running evaluation for iteration {iter} response {idx}:\n {real_str}')
                    lines = real_str.split("\n")
                    for line in lines:
                        if "total_reward" in line:
                            realworld_rewards.append(float(line.split(":")[1].strip()))
                            break

                    for line in lines:
                        if "total_survival_time" in line:
                            realworld_survival_times.append(float(line.split(":")[1].strip()))
                            break

                    logging.info(f"Iteration {iter}: Realworld evaluation for code path {code_path} completed successfully!")

                command = f"python3 -u {ROOT_DIR}/src/eval/realworld_reward_original.py --data_path={data_path}"
                command = command.split(" ")
                real_eval_original_filepath = f"env_{env_name}_iter{iter}_response{idx}_real_eval_original.txt"
                with open(real_eval_original_filepath, 'w') as f:
                    process = subprocess.Popen(command, stdout=f, stderr=f)
                    process.wait()

                real_original_str = file_to_string(real_eval_original_filepath)
                realworld_original_feedbacks.append(f'Human engineered reward function evaluation in reality for iteration {iter} response {idx}:\n {real_original_str}')

                contents[idx] += real_feedback.format(sim_time=cfg.env.tracking_duration)
                contents[idx] += realworld_feedbacks[i]
                contents[idx] += realworld_original_feedbacks[i]

        # Select best sample based on both simulation and real-world evaluation
        test_rewards = np.array(deployment_rewards)
        if cfg.test_realworld:
            test_rewards += np.array(realworld_rewards)
        best_idx_id = np.argmax(test_rewards)
        best_sample_idx = top_idx[best_idx_id]
        best_content = contents[best_sample_idx]
        max_success = successes[best_sample_idx]
        max_success_reward_correlation = reward_correlations[best_sample_idx]
        execute_rate = np.sum(np.array(successes) >= 0.) / cfg.sample

        # Update the best Eureka Output
        if max_success > max_success_overall:
            max_success_overall = max_success
            max_success_reward_correlation_overall = max_success_reward_correlation
            max_reward_code_path = code_paths[best_sample_idx]

        execute_rates.append(execute_rate)
        max_successes.append(max_success)
        max_successes_reward_correlation.append(max_success_reward_correlation)
        best_code_paths.append(code_paths[best_sample_idx])

        logging.info(f"Iteration {iter}: Max Success: {max_success}, Execute Rate: {execute_rate}, Max Success Reward Correlation: {max_success_reward_correlation}")
        logging.info(f"Iteration {iter}: Best Generation ID: {best_sample_idx}")
        logging.info(f"Iteration {iter}: GPT Output Content:\n" +  responses[best_sample_idx]["message"]["content"] + "\n")
        logging.info(f"Iteration {iter}: User Content:\n" + best_content + "\n")

        # Plot the success rate
        fig, axs = plt.subplots(2, figsize=(6, 6))
        fig.suptitle(f'{env_name}')

        x_axis = np.arange(len(max_successes))

        axs[0].plot(x_axis, np.array(max_successes))
        axs[0].set_title("Max Success")
        axs[0].set_xlabel("Iteration")

        axs[1].plot(x_axis, np.array(execute_rates))
        axs[1].set_title("Execute Rate")
        axs[1].set_xlabel("Iteration")

        fig.tight_layout(pad=3.0)
        plt.savefig('summary.png')
        np.savez('summary.npz', max_successes=max_successes, execute_rates=execute_rates, best_code_paths=best_code_paths, max_successes_reward_correlation=max_successes_reward_correlation)

        if len(messages) == 2:
            messages += [{"role": "assistant", "content": responses[best_sample_idx]["message"]["content"]}]
            messages += [{"role": "user", "content": best_content}]
        else:
            assert len(messages) == 4
            messages[-2] = {"role": "assistant", "content": responses[best_sample_idx]["message"]["content"]}
            messages[-1] = {"role": "user", "content": best_content}

        # Save dictionary as JSON file
        with open('messages.json', 'w') as file:
            json.dump(messages, file, indent=4)
    
    if max_reward_code_path is None: 
        logging.info("All iterations of code generation failed, aborting...")
        logging.info("Please double check the output env_iter*_response*.txt files for repeating errors!")
        exit()
    logging.info(f"Task: {task_name}, Max Training Success {max_success_overall}, Correlation {max_success_reward_correlation_overall}, Best Reward Code Path: {max_reward_code_path}")

    best_reward = file_to_string(max_reward_code_path)
    with open(reward_dest, 'w') as file:
        file.writelines(best_reward + '\n')
    
    logging.info(f"Best Reward Function:\n {best_reward}")
    logging.info(f"Best Reward Function saved at {reward_dest}")
    logging.info("Training completed successfully!")

    # remove cache directory
    shutil.rmtree(cache_dir)
    logging.info("Cache directory removed!")

if __name__ == '__main__':
    main()