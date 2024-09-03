import subprocess
import torch
import os

def get_freest_gpu():
    # Run nvidia-smi command and get the output
    # get gpu indices from CUDA_VISIBLE_DEVICES
    cuda_visible_devices = os.environ.get('CUDA_VISIBLE_DEVICES')
    if cuda_visible_devices is None:
        gpu_indices = list(range(torch.cuda.device_count()))
    else:
        print('CUDA_VISIBLE_DEVICES:', cuda_visible_devices)
        gpu_indices = list(map(int, cuda_visible_devices.split(',')))

    result = subprocess.run(['nvidia-smi', '--query-gpu=memory.used,memory.total', '--format=csv,nounits,noheader'], stdout=subprocess.PIPE)
    output = result.stdout.decode('utf-8').strip().split('\n')

    # Parse the output to get memory usage
    memory_usage = []
    for i in gpu_indices:
        used, total = map(int, output[i].split(','))
        memory_usage.append((used, total))

    # Find the GPU with the least memory usage
    freest_gpu = min(gpu_indices, key=lambda i: memory_usage[gpu_indices.index(i)][0] / memory_usage[gpu_indices.index(i)][1])
    return freest_gpu, len(gpu_indices)

def set_freest_gpu():
    # Set the environment variable to include the specified GPU indices
    freest_gpu, _ = get_freest_gpu()
    torch.cuda.set_device(freest_gpu)
    return freest_gpu