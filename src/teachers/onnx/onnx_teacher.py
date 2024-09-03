import torch
import numpy as np
import onnxruntime as ort
from concurrent.futures import ThreadPoolExecutor
import os

file_path = __file__
file_root = os.path.dirname(file_path)
model_path = os.path.join(file_root, 'model.onnx')

global_session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])
input_name = global_session.get_inputs()[0].name

def teacher(input_tensor: torch.Tensor, max_workers=4) -> torch.Tensor:
    input_data = input_tensor.cpu().numpy()

    def run_model(data):
        return [global_session.run(None, {input_name: data[i]})for i in range(len(data))]
    
    batch_size = 10 
    batches = [input_data[i:i + batch_size] for i in range(0, len(input_data), batch_size)]

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        results = executor.map(run_model, batches)
    output_data = np.concatenate([item for sublist in list(results) for item in sublist])
    output_tensor = torch.from_numpy(output_data).to(input_tensor.device)
    
    return output_tensor