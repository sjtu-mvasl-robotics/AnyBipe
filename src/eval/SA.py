import numpy as np
def sa_func(str):
    lines = str.split('\n')
    criter = {}
    for line in lines: 
        try:
            name, value = line.split(' ')
            criter[name] = float(value)
        except:
            pass
    
    # implement your criteria here
    return criter['total_survival_time'] >= 25.0 and criter['total_reward'] >= -20.0