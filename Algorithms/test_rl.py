import matplotlib.pyplot as plt
import numpy as np

from q_learning.agent import QLearningTrainer
from q_learning.rl_env import RLEnvironment
from ant_env.environment import AntennaEnvironmentSim
from q_learning.test import test_agent

import matplotlib as mpl

mpl.rcParams["font.family"] = "serif"
mpl.rcParams["font.family"] = "Times New Roman"
mpl.rcParams["axes.titlesize"] = 12
mpl.rcParams["axes.labelsize"] = 12
mpl.rcParams["legend.fontsize"] = 11
mpl.rcParams["xtick.labelsize"] = 10
mpl.rcParams["ytick.labelsize"] = 10


# Initialize environment
antenna_env = AntennaEnvironmentSim()
env_wrapper = RLEnvironment(
    antenna_env=antenna_env,
    samples=20,
)

# Load trained Q-table
agent = QLearningTrainer()
agent.Q = np.load("q_table.npy")
agent.epsilon = 0.0  # pure exploitation

# Run test
log_data, path = test_agent(
    agent,
    env_wrapper,
    max_steps=300,
    samples=env_wrapper.samples,
)
print(log_data)

# Plot RL path
pan = [p["pan"] for p in path]
tilt = [p["tilt"] for p in path]
best_point = log_data["metrics"]["best_point"]

plt.figure(figsize=(6, 6))
plt.plot(tilt, pan, "-o", label="RL Path")
plt.scatter(best_point["tilt"], best_point["pan"], c="red", s=100, label="Best Point")
plt.xlabel("Tilt (deg)")
plt.ylabel("Pan (deg)")
plt.title("RL Q-learning Path")
plt.legend()
plt.grid(True)
plt.show()

# RSSI convergence
rssi_values = [p["rssi"] for p in path]
plt.figure(figsize=(6, 4))
plt.plot(range(len(rssi_values)), rssi_values, "-o")
plt.xlabel("Step")
plt.ylabel("RSSI (dBm)")
plt.title("RL RSSI Convergence")
plt.grid(True)
plt.show()
