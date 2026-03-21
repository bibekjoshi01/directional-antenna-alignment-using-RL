import time
import matplotlib.pyplot as plt
from baseline_algos.hill_climbing import HillClimb2D
from ant_env.environment import AntennaEnvironmentSim

import matplotlib as mpl

mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.family'] = 'Times New Roman'
mpl.rcParams['axes.titlesize'] = 12
mpl.rcParams['axes.labelsize'] = 12
mpl.rcParams['legend.fontsize'] = 11
mpl.rcParams['xtick.labelsize'] = 10
mpl.rcParams['ytick.labelsize'] = 10

env = AntennaEnvironmentSim()

hc = HillClimb2D(
    env,
    pan_range=(0, 360),
    tilt_range=(60, 120),
    step_size=2,
    samples_per_point=20,
    max_iters=200,
    patience=15,
    settle_time=0.0,
    init_point=(150, 70),  # deliberately bad start
)

start_time = time.time()
print("Performing the scan ....")
best_rssi, best_point, path = hc.run_scan()
end_time = time.time()
convergence_time = end_time - start_time

log_data = {
    "algorithm": "Hill Climbing",
    "params": {
        "pan_range": (0, 360),
        "tilt_range": (60, 120),
        "step_size": hc.step,
        "samples_per_point": hc.samples,
        "max_iters": hc.max_iters,
        "patience": hc.patience,
        "init_point": hc.init_point,
    },
    "metrics": {
        "best_rssi": best_rssi,
        "best_point": best_point,
        "convergence_time_sec": convergence_time,
        "num_steps_taken": len(path),
        "total_samples": len(path) * hc.samples,
    },
}

print(log_data)

# ---- plot path ----
pan = [p["pan"] for p in path]
tilt = [p["tilt"] for p in path]

plt.figure(figsize=(6, 6))
plt.plot(tilt, pan, "-o", label="Scan Path")
plt.scatter(best_point["tilt"], best_point["pan"], c="red", s=100, label="Best Point")
plt.xlabel("Tilt (deg)")
plt.ylabel("Pan (deg)")
plt.title("Hill Climbing Path to Best RSSI")
plt.legend()
plt.grid(True)
plt.show()

rssi_values = [p["rssi"] for p in path]

plt.figure(figsize=(6, 4))
plt.plot(range(len(rssi_values)), rssi_values, "-o")
plt.xlabel("Step")
plt.ylabel("Measured RSSI (dBm)")
plt.title("RSSI Convergence During Hill Climbing")
plt.grid(True)
plt.show()
