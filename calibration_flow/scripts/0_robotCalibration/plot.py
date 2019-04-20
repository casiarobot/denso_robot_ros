import numpy as np
import yaml
import matplotlib.pyplot as plt

if __name__ == "__main__":
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
    J1_PATH = path['robotCalibration'] + 'goal/j1.yaml'
    
    with open(J1_PATH) as f:
        J1p = yaml.load(f)
    
    J1v = np.gradient(J1p)
    plt.figure()
    plt.plot(J1p, 'r.')
    plt.figure()
    plt.plot(J1v)
    plt.show()