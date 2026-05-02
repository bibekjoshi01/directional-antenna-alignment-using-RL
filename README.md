Minor Project — Antenna Positioning and RL Baselines
=====================================================

Short description
-----------------
Code and experiments for comparing classical and RL-based antenna positioning algorithms. Includes simulators, baseline algorithms, RL training/evaluation, and receiver/transmitter examples.

Contents
--------
- `Algorithms/` — Python implementations and tests (exhaustive scan, hill-climbing, stochastic, RL training/evaluation).
- `Algorithms/ant_env/` — environment and RSSI simulation utilities.
- `Algorithms/q_learning/` — Q-learning agent and training/testing scripts.
- `ValuesCollected/` — experiment outputs and values used for analysis.
- `Receiver/` & `Transmitter/` — C++ example code and deployment snippets.
- `Reports/` & `Papers/` — documentation, diagrams, and references.

Requirements
------------
Use Python 3.8+ and install dependencies from `Algorithms/requirements.txt`.

Quick start
-----------
1. Create a Python environment (recommended): `python -m venv .venv && source .venv/bin/activate`
2. Install dependencies: `pip install -r Algorithms/requirements.txt`
3. Run tests or experiments in `Algorithms/`, e.g. `python Algorithms/test_rl.py`.

Notes
-----
- Scripts and experiments are primarily in `Algorithms/` — check individual README files there for details.
- C++ examples in `Receiver/` and `Transmitter/` are standalone and may require platform-specific toolchains.

License
-------
No license specified.

Contact
-------
Project owner: bibekjoshi01 (local workspace)