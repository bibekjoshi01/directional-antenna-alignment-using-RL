{
    "algorithm": "Exhaustive Scan",
    "params": {
        "pan_range": (0, 360),
        "tilt_range": (60, 120),
        "pan_step": 10,
        "tilt_step": 5,
        "fine_step": 2,
        "samples_per_point": 20,
        "settle_time": 0.0,
    },
    "metrics": {
        "best_rssi": -30.203245189814577,
        "best_point": {
            "pan": np.int64(88),
            "tilt": np.int64(90),
            "rssi": -30.203245189814577,
        },
        "convergence_time_sec": 67.68175530433655,
        "num_steps_taken": 945,
        "total_samples": 18900,
    },
}


{
    "algorithm": "Hill Climbing",
    "params": {
        "pan_range": (0, 360),
        "tilt_range": (60, 120),
        "step_size": 2,
        "samples_per_point": 20,
        "max_iters": 200,
        "patience": 15,
        "init_point": (40, 70),
    },
    "metrics": {
        "best_rssi": -30.129623365483305,
        "best_point": {"pan": 90, "tilt": 86, "rssi": -30.129623365483305},
        "convergence_time_sec": 6.513987064361572,
        "num_steps_taken": 42,
        "total_samples": 840,
    },
}


{
    "algorithm": "RL Q-learning",
    "params": {"max_steps": 100},
    "metrics": {
        "best_rssi": -29.824406543397423,
        "best_point": {"pan": 90, "tilt": 90},
        "convergence_time_sec": 5.51346655,
        "num_steps_taken": 100,
        "total_samples": 2000,
    },
}
