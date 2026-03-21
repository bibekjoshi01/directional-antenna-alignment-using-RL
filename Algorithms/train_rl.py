from q_learning.agent import QLearningTrainer
from q_learning.rl_env import RLEnvironment
from ant_env.environment import AntennaEnvironmentSim
from q_learning.train import train_agent

import matplotlib as mpl

mpl.rcParams["font.family"] = "serif"
mpl.rcParams["font.family"] = "Times New Roman"
mpl.rcParams["axes.titlesize"] = 14
mpl.rcParams["axes.labelsize"] = 14
mpl.rcParams["legend.fontsize"] = 14
mpl.rcParams["xtick.labelsize"] = 12
mpl.rcParams["ytick.labelsize"] = 12


antenna_env = AntennaEnvironmentSim()
env_wrapper = RLEnvironment(antenna_env=antenna_env)
agent = QLearningTrainer()

history = train_agent(agent, env_wrapper, n_episodes=1000, max_steps=100, patience=10)

# plotting
import matplotlib.pyplot as plt
import numpy as np

history = np.array(history)

# Compute cumulative max RSSI per episode
cumulative_best = np.maximum.accumulate(history)

# Moving average for smoothing
window = 10
moving_avg = np.convolve(history, np.ones(window) / window, mode="valid")

# ----Reward per episode ----
plt.figure(figsize=(10, 5))
plt.plot(history, label="Reward per episode", alpha=0.5)
plt.plot(
    range(window - 1, len(history)),
    moving_avg,
    label=f"Moving avg ({window})",
    color="orange",
)
plt.xlabel("Episode")
plt.ylabel("Total Reward")
plt.title("RL Training Reward per Episode")
plt.legend()
plt.grid(True)
plt.show()

# --- Cumulative best RSSI ----
plt.figure(figsize=(10, 5))
plt.plot(cumulative_best, color="green")
plt.xlabel("Episode")
plt.ylabel("Cumulative Best Reward / RSSI")
plt.title("Cumulative Best RSSI Over Episodes")
plt.grid(True)
plt.show()
