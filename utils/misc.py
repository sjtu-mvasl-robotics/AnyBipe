import subprocess
import os
import json
import logging
import re
import ast
import importlib.util
import sys

from utils.extract_task_code import file_to_string

def set_freest_gpu():
    freest_gpu = get_freest_gpu()
    os.environ['CUDA_VISIBLE_DEVICES'] = str(freest_gpu)

def get_freest_gpu():
    # Note: if this line breaks, you can provide an absolute path to gpustat instead
    sp = subprocess.Popen(['gpustat', '--json'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out_str, _ = sp.communicate()
    gpustats = json.loads(out_str.decode('utf-8'))
    freest_gpu = min(gpustats['gpus'], key=lambda x: x['memory.used'])
    return freest_gpu['index']

def load_prompts(path, name):
    return file_to_string(f"{path}/{name}.txt")

def filter_traceback(s):
    lines = s.split('\n')
    filtered_lines = []
    for i, line in enumerate(lines):
        if line.startswith('Traceback'):
            for j in range(i, len(lines)):
                if "Set the environment variable HYDRA_FULL_ERROR=1" in lines[j]:
                    break
                filtered_lines.append(lines[j])
            return '\n'.join(filtered_lines)
    return ''  # Return an empty string if no Traceback is found

def block_until_training(rl_filepath, success_keyword, failure_keyword, log_status=False, iter_num=-1, response_id=-1):
    # Ensure that the RL training has started before moving on
    while True:
        rl_log = file_to_string(rl_filepath)
        if "Traceback" in rl_log or "Learning finished" in rl_log:
            if log_status and "Learning finished" in rl_log:
                logging.info(f"Iteration {iter_num}: Code Run {response_id} training finished!")
            if log_status and "Traceback" in rl_log:
                logging.info(f"Iteration {iter_num}: Code Run {response_id} execution error!")
            break


def construct_run_log(stdout_str):
    # Construct the run log
    run_log = {}
    lines = stdout_str.split('\n')
    not_found = True
    for i, line in enumerate(lines):
        # skip the first few lines until "Learning iteration"
        if not_found:
            if "Learning iteration" in line:
                not_found = False
            continue

        # see if it is "iteration" line
        if "Learning iteration" in line:
            # format: Learning iteration 0/1000
            iter, total_iter = line.strip().strip("Learning iteration").split("/")
            run_log["iterations"] = run_log.get("iterations", []) + [int(iter) + 1]


        # see if line has : in it
        if ":" in line:
            if "Computation" in line or "Iteration" in line or "ETA" in line:
                continue
            elif "Current save path" in line: # training finished
                run_log["save_path"] = line.split(":")[1].strip()
                break
            try:
                key, val = line.split(":")
            except:
                continue
            key = key.strip()
            val = val.strip()
            # if val is not a number, skip
            try:
                float(val)
            except:
                continue
            if "Mean" in key:
                key = key.replace("Mean", "")
                key = key.strip()
            if "episode" in key:
                key = key.replace("episode", "")
                key = key.strip()
            run_log[key] = run_log.get(key, []) + [float(val)]
    
    run_log["gpt_reward"] = []
    #run_log["gt_reward"] = []
    # check if "rew_success" is present in the run_log
    if "rew_success" in run_log:
        for i in range(len(run_log["rew_success"])):
            cur_sum = 0
            for key in run_log:
                if "rew" in key and "gpt" not in key and "success" not in key and "total" not in key:
                    cur_sum += run_log[key][i]
            run_log["gpt_reward"].append(cur_sum)
            #run_log["gt_reward"].append(cur_sum)
    return run_log

def construct_run_log_gt(stdout_str):
    # Construct the run log (ground truth reward)
    run_log = {}
    lines = stdout_str.split('\n')
    not_found = True
    for i, line in enumerate(lines):
        if not_found:
            if "Learning iteration" in line:
                not_found = False
            continue
        # see if it is "iteration" line
        if "Learning iteration" in line:
            # format: Learning iteration 0/1000
            iter, total_iter = line.strip().replace("Learning iteration", "").split("/")
            run_log["iterations"] = run_log.get("iterations", []) + [int(iter) + 1]
            continue
        # see if line has : in it
        if ":" in line:
            if "Computation" in line or "Iteration" in line or "ETA" in line:
                continue
            elif "Current save path" in line: # training finished
                run_log["save_path"] = line.split(":")[1].strip()
                break
            try:
                key, val = line.split(":")
            except:
                continue
            key = key.strip()
            val = val.strip()
            # if val is not a number, skip
            try:
                float(val)
            except:
                continue
            if "Mean" in key:
                key = key.replace("Mean", "")
                key = key.strip()
            if "episode" in key:
                key = key.replace("episode", "")
                key = key.strip()
            run_log[key] = run_log.get(key, []) + [float(val)]
    
    #run_log["gpt_reward"] = []
    run_log["gt_reward"] = []
    # check if "rew_success" is present in the run_log
    if "rew_success" in run_log:
        for i in range(len(run_log["rew_success"])):
            cur_sum = 0
            for key in run_log:
                if "rew" in key and "gt" not in key and "success" not in key and "total" not in key:
                    cur_sum += run_log[key][i]
            #run_log["gpt_reward"].append(cur_sum)
            run_log["gt_reward"].append(cur_sum)
    return run_log

def find_export_dir(stdout_str):
    lines = stdout_str.split('\n')
    for line in lines:
        if "Exported policy as onnx script to:" in line:
            return line.split(":")[1].strip()
    return None

def model_deployment_test(model_path, env_name, gym_path, deploy_path, script_path, monitor_py, tracking_duration, log_dir, ROOT_DIR):
    prep = model_path.split("/")
    model_name = prep[-2] + "_" + prep[-1].split(".")[0]
    # run deployment script
    deployment_script = f"python3 {gym_path}/legged_gym/scripts/export_policy_as_onnx.py --ckpt_path {model_path} --task {env_name}"
    deployment_script = deployment_script.split(" ")
    log_file = f"{log_dir}/{model_name}_deployment.log"
    with open(log_file, "w") as f:
        process = subprocess.Popen(deployment_script, stdout=f, stderr=f)
        process.wait()

    # find the export directory
    export_dir = find_export_dir(file_to_string(log_file))
    if export_dir is None:
        return None
    
    # copy the original policy file to backup
    os.system(f"cp {deploy_path}/policy.onnx {deploy_path}/policy_original.onnx")

    # copy the onnx file to the deployment path
    os.system(f"cp {export_dir} {deploy_path}/policy.onnx")

    # run the monitoring script

    monitoring_script = f"python3 {script_path}/{monitor_py} --tracking_duration {tracking_duration} --project_path {ROOT_DIR}/biped_ws --export_path {log_dir}/exported_data/"
    monitoring_script = monitoring_script.split(" ")
    log_file = f"{log_dir}/{model_name}_monitoring.log"
    with open(log_file, "w") as f:
        process = subprocess.Popen(monitoring_script, stdout=f, stderr=f)
        process.wait()
    
    log_str = file_to_string(log_file)
    for line in log_str.split("\n"):
        if "Data exported to " in line:
            return line.split("Data exported to ")[1].strip()
    return None


def replace_first_dim_with_i(expression):
    
    pattern = r'(\w+(?:\.\w+)*)\[\s*([^,\]]*)\s*,(.*)\]'

    def replacer(match):
        array_name = match.group(1)
        remaining_dims = match.group(3)
        
        # Replace the first dimension with 'idx:idx+1'
        return f'{array_name}[idx:idx+1, {remaining_dims}]'
    
    return re.sub(pattern, replacer, expression)

def get_reward_scale(lines):
    return_terms = lines[-1].split("return")[1].strip()
    if "#" in return_terms:
        return_terms = return_terms.split("#")[0].strip()
    if "*" in return_terms:
        # check if one of the term before/after "*" is a number
        terms = return_terms.split("*")
        for term in terms:
            try:
                reward_term_scale = float(term)
                break
            except:
                reward_term_scale = 1.0
    else:
        reward_term_scale = 1.0

    return reward_term_scale

def convert_to_realworld_reward(reward_str, func_map):
    # check if reward_str begin with def _reward_ (ignoring whitespace)
    if not reward_str.strip().startswith("def _reward_"):
        return None
    # check if last line starts with return
    lines = reward_str.strip().split("\n")
    if not lines[-1].strip().startswith("return"):
        return None
    
    reward_func_name = lines[0].split("def _reward_")[1].split("(")[0]
    # special case for survival
    if "survival" in reward_func_name:
        # check if return term contains "*"
        reward_term_scale = get_reward_scale(lines)
        return """
    def _reward_survival(self, idx):
        return {reward_term_scale} * self.env.dt[idx]

    """.format(reward_term_scale=reward_term_scale)

    elif "dof_acc" in reward_func_name:
        reward_term_scale = get_reward_scale(lines)
        return """
    def _reward_dof_acc(self, idx):
        return {reward_term_scale} * torch.sum(torch.square(self.env.ddq[idx:idx+1,:]), dim=1)
    """.format(reward_term_scale=reward_term_scale)

    # check if function has "env = self.env" line
    global_set_env = False
    for line in lines:
        if "env = self.env" in line:
            #print("env = self.env found in line")
            global_set_env = True
            break

    reward_str = reward_str.replace("(self)", "(self, idx)")

    lines = reward_str.split("\n")

    for i, line in enumerate(lines):
        if "env = self.env" in line:
            continue
        if "env.cfg.rewards" in line:
            #print("env.cfg.rewards found in line {}".format(i))
            lines[i] = line.replace("env.cfg.rewards", "self.reward") if global_set_env else line.replace("self.env.cfg.rewards", "self.reward")
            #print(lines[i])
            line = lines[i]
        
        if '#' in line:
            line = line.split('#')[0]

        # match all terms that begin with self.env. (including array slicing)
        if global_set_env:
            env_terms = re.findall(r'env(?:\.\w+)*(?:\[\s*[\d:,\s-]*\s*\])?', line)
        else:
            env_terms = re.findall(r'self\.env(?:\.\w+)*(?:\[\s*[\w\d:,\s-]*\s*\])?', line)
        
        # print(line)

        for term in env_terms:
            term_name = term.split(".")[-1]
            if "shape" in term:
                term_name = term.split(".")[-2]
            # check if term is an array slicing
            slicing = False
            if "[" in term:
                term_name = term_name.split("[")[0]
                slicing = True

            if term_name not in func_map.keys():
                # unconvertable
                return None
            new_term = term.replace(term_name, func_map[term_name])
            if slicing:
                new_term = replace_first_dim_with_i(new_term)
            lines[i] = lines[i].replace(term, new_term)
    # merge the lines
    reward_str = "\n".join(lines)

    #print(reward_str)

    
    return reward_str

def convert_reward_file(src_file_path, template_file_path, func_map):
    target_file_path = template_file_path.replace("_template", "")
    source_str = file_to_string(src_file_path)
    target_str = file_to_string(template_file_path)
    
    functions = []
    
    tree = ast.parse(source_str)
    
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            function_code = ast.get_source_segment(source_str, node)
            # add two indentations to the function code
            function_code = "\n".join(["    " + line for line in function_code.split("\n")])
            functions.append(function_code)

    converted_functions = []
    return_str = ""
    for function in functions:
        #print("Converting function")
        #print(function)
        converted_function = convert_to_realworld_reward(function, func_map)
        if converted_function is None:
            return_str += "Failed to convert function with name {}\n".format(function.split("\n")[0].split("(")[0])
            continue
        converted_functions.append(converted_function)
        #print("successfully converted function with name {}".format(function.split("\n")[0].split("("[0])))

    functions_str = "\n\n".join(converted_functions)
    target_str = target_str.replace("# INSERT REMODULARIZED REWARD HERE", functions_str)
    with open(target_file_path, 'w') as file:
        file.write(target_str)

    return return_str

def import_module_from_file(script_path, function_name):
    script_path = os.path.abspath(script_path)
    
    module_name = os.path.splitext(os.path.basename(script_path))[0]
    module_dir = os.path.dirname(script_path)
    
    if module_dir not in sys.path:
        sys.path.append(module_dir)

    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    
    if hasattr(module, function_name):
        func = getattr(module, function_name)
        return func
    else:
        raise AttributeError(f"The function {function_name} does not exist in {script_path}.")

    
