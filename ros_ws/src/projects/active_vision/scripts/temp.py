import numpy as np

trial = []
for i in range(20):
    trial.append(i)
    if(len(trial) > 10):
        trial = trial[1:11]
    print(trial)