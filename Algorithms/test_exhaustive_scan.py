import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib as mpl

mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.family'] = 'Times New Roman'
mpl.rcParams['axes.titlesize'] = 12
mpl.rcParams['axes.labelsize'] = 12
mpl.rcParams['legend.fontsize'] = 11
mpl.rcParams['xtick.labelsize'] = 10
mpl.rcParams['ytick.labelsize'] = 10

from baseline_algos.exhaustive_scan import ExhaustiveScan2D
from ant_env.environment import AntennaEnvironmentSim

env = AntennaEnvironmentSim()

scanner = ExhaustiveScan2D(
    env,
    pan_range=(0, 360),
    tilt_range=(60, 120),
    pan_step=10,
    tilt_step=5,
    fine_step=2,
    samples_per_point=20,
    settle_time=0.0,
)

start_time = time.time()
print("Performing the scan ....")
best_rssi, best_point = scanner.run_scan()
end_time = time.time()
convergence_time = end_time - start_time

# Total number of steps sampled
pan_vals, tilt_vals, rssi_grid = scanner.get_rssi_grid()
num_steps_taken = rssi_grid.size
total_samples = num_steps_taken * scanner.samples

log_data = {
    "algorithm": "Exhaustive Scan",
    "params": {
        "pan_range": (0, 360),
        "tilt_range": (60, 120),
        "pan_step": scanner.pan_step,
        "tilt_step": scanner.tilt_step,
        "fine_step": scanner.fine_step,
        "samples_per_point": scanner.samples,
        "settle_time": scanner.settle_time,
    },
    "metrics": {
        "best_rssi": best_rssi,
        "best_point": best_point,
        "convergence_time_sec": convergence_time,
        "num_steps_taken": num_steps_taken,
        "total_samples": total_samples,
    },
}

print(log_data)

X, Y = np.meshgrid(tilt_vals, pan_vals)
plt.figure(figsize=(10, 6))
plt.contourf(X, Y, rssi_grid, levels=50, cmap="viridis")
plt.colorbar(label="RSSI (dBm)")

plt.scatter(
    best_point["tilt"],
    best_point["pan"],
    color="red",
    s=80,
    label=f"Best: {round(best_rssi, 2)}",
)
plt.xlabel("Tilt (deg)")
plt.ylabel("Pan (deg)")
plt.title("RSSI Surface (Exhaustive Scan)")
plt.legend()
plt.grid(True)
plt.show()
