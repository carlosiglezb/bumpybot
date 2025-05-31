import numpy as np

def load_BB_outline(filepath='/home/hcrl-bumpybot/bumpybot_ws/src/bumpybot_torque_contact/cfg/BB_outline.csv'):
    return np.loadtxt(filepath, delimiter=',')
