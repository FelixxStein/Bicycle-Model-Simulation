# MPPI Racing
Sampling-based model predictive control for an autonomous race car on real F1 tracks
## Features
## Features

### 🚗 Vehicle Model
| Feature | Description |
|---|---|
| Dynamic single-track model | State: position, yaw, vx, vy, yaw rate, steering angle. Explicit Euler integration at dt = 0.01 s |
| Magic Formula tires | Nonlinear lateral tire forces that saturate at the grip limit |
| Friction circle | Longitudinal and lateral forces share one grip budget, so braking or accelerating in a corner costs lateral grip |
| Load transfer, downforce & drag | Braking shifts load to the front axle, downforce grows with v², plus aerodynamic drag and rolling resistance |
| Kinematic/dynamic blending | Smooth transition to a kinematic model at low speed (3–8 m/s), where tire models become ill-conditioned |
| Steering rate limit | The wheels can't "teleport": max. 1 rad/s steering rate with a 0.05 s time constant |
| Physically consistent sliding | Brakes act as friction against the direction of motion, slip angles are computed in the wheel frame (valid for sliding and rolling backwards) |
| Batch simulation | The same code simulates one car or 1000 MPPI rollouts at once via NumPy broadcasting |

| Parameter | Value |
|---|---|
| Mass | 800 kg |
| Wheelbase | 3.5 m |
| Friction coefficient | 1.6 |
| Max. acceleration / braking | 11 m/s² / 40 m/s² |
| Max. steering angle | 0.35 rad |

### 🏁 Track
| Feature | Description |
|---|---|
| Real F1 circuits | Hockenheim and Spielberg from the TUMFTM racetrack database (centerline + track widths) |
| Speed profile | Maximum cornering speed from curvature and grip (incl. downforce), plus a backward pass for braking zones |
| Crash detection | A lap ends if the car gets closer than 0.5 m to the track edge or faces more than 90° against the driving direction (spin) |

### 🎯 Baseline Controller
| Feature | Description |
|---|---|
| Pure Pursuit + PID | Steering towards a speed-dependent lookahead point on the centerline, speed tracking via PID on a planned speed profile |

### 🧠 MPPI Controller
| Feature | Description |
|---|---|
| Sampling-based MPC | Every 0.05 s, 1000 random control sequences are simulated ~3 s ahead, rated by a cost function and blended into one weighted-average plan |
| Correlated noise | AR(1)-filtered noise (α = 0.9) produces smooth control sequences instead of jitter |
| Speed-dependent steering noise | Steering noise scales with 1/v², keeping lateral excitation reasonable at high speed |
| Candidate plans | The previous plan, a full-throttle and a full-brake variant are always evaluated alongside the random samples |
| Cost function | Progress along the track, crash penalty (off-track or spin), terminal speed vs. speed profile, speed-dependent distance-to-edge penalty, steering smoothness |
| Adaptive temperature | Weighting temperature scales with the cost spread of non-crashed plans, making the controller robust to the cost scale |
| Warm start | The previous plan is shifted by one step and reused as the base for the next iteration |

### 🔧 Hyperparameter Tuning
| Feature | Description |
|---|---|
| Cross-Entropy Method | 9 MPPI parameters sampled in log space; the best sets of each generation define the next sampling distribution |
| Multi-seed evaluation | Every parameter set is evaluated on several random seeds to reward robustness, not luck |
| Progress-aware score | Finished laps are scored by lap time, crashed laps by remaining distance, so "further" still counts as better |
| Parallel execution | Laps run in parallel via `multiprocessing` |
| Warm start & logging | Optional start from a known parameter set; every lap is logged to CSV |

### 🎬 Visualization
| Feature | Description |
|---|---|
| Pygame replay | Car sprite, MPPI rollouts colored by weight (cyan) or crash (red), planned path (yellow), automatic framing camera |
| Controller-agnostic | Also replays baseline laps (with a look-ahead camera instead of rollouts) |
| Speed plot | Racing line colored by speed (matplotlib) |

### 🛠️ Engineering
| Feature | Description |
|---|---|
| Package structure | `racesim/` library with thin entry-point scripts |
| Common controller interface | All controllers implement `compute_control(state, progress, index)`; one shared `simulate_lap` for runs and tuning |
| CLI & configs | `argparse` entry points, parameter sets stored as JSON |
| Physics tests | `pytest` suite: standstill braking, rolling backwards, sideways sliding, batch consistency |
## How it works

### 1. Vehicle model

The car is a dynamic single-track model with state

$$
\mathbf{x} = [x,\ y,\ \psi,\ v_x,\ v_y,\ r,\ \delta]
$$

(position, yaw, longitudinal/lateral velocity, yaw rate, steering angle) and inputs $\mathbf{u} = [\delta_{target},\ a]$ (steering target, acceleration/braking). It is integrated with explicit Euler at $\Delta t = 0.01$ s.

**Tires.** Lateral forces follow a simplified Magic Formula, with the peak force set by the axle load:

$$
F_y = \mu F_z \,\sin\!\big(C \arctan(B\alpha)\big)
$$

The front slip angle $\alpha$ is computed in the wheel frame, so it stays correct for large angles, sliding and even rolling backwards.

**Friction circle.** Longitudinal and lateral forces share one grip budget:

$$
F_y \leftarrow F_y \sqrt{1 - \left(\frac{F_x}{\mu F_z}\right)^2}
$$

**Equations of motion** (body frame):

$$
\begin{aligned}
m(\dot v_x - v_y r) &= F_{x,f}\cos\delta - F_{y,f}\sin\delta + F_{x,r} - F_{drag} - F_{roll} \\
m(\dot v_y + v_x r) &= F_{x,f}\sin\delta +
## Acknowledgements
Track data sourced from [TUMFTM/racetrack-database](https://github.com/TUMFTM/racetrack-database).
