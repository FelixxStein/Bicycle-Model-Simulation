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
m(\dot v_y + v_x r) &= F_{x,f}\sin\delta + F_{y,f}\cos\delta + F_{y,r} \\
I_z \dot r &= l_f\,(F_{x,f}\sin\delta + F_{y,f}\cos\delta) - l_r F_{y,r}
\end{aligned}
$$

Axle loads include static weight distribution, longitudinal load transfer and aerodynamic downforce ($\propto v^2$). Brakes act like friction (always against the direction of motion), drive force acts on the rear axle only.

**Steering actuator.** The wheels follow the steering target with a time constant and a rate limit:

$$
\dot\delta = \mathrm{clip}\!\left(\frac{\delta_{target} - \delta}{\tau},\ -\dot\delta_{max},\ \dot\delta_{max}\right)
$$

**Low-speed blending.** Below 8 m/s the dynamic model is smoothly blended with a kinematic model (fully kinematic below 3 m/s), because tire slip angles become ill-conditioned near standstill.

### 2. Track & speed profile

Tracks come from the TUMFTM racetrack database (centerline + left/right widths). From the curvature $\kappa$ a reference speed profile is computed. The maximum cornering speed balances lateral acceleration against grip incl. downforce ($k_L = \tfrac12\rho c_L A$):

$$
v_{corner}^2 = \frac{\mu g}{|\kappa| - \mu k_L / m}
$$

A backward pass then adds braking zones, so the profile is always reachable with the available braking:

$$
v_i = \min\!\left(v_{corner,i},\ \sqrt{v_{i+1}^2 + 2\,a_{brake}(v_{i+1})\,\Delta s}\right)
$$

### 3. MPPI controller

Model Predictive Path Integral control is a sampling-based MPC: instead of solving an optimization problem, it simulates many possible futures and blends the good ones. Every 0.05 s it runs this loop:

1. **Sample** $N = 1000$ control sequences around the current plan $U$ (horizon ≈ 3 s):
   $V_k = U + \varepsilon_k$, with time-correlated noise
   $\varepsilon_t = \alpha\,\varepsilon_{t-1} + \sqrt{1-\alpha^2}\,w_t$ ($\alpha = 0.9$) for smooth inputs.
   The steering noise shrinks with speed, $\sigma_\delta \propto 1/v^2$. Three candidate plans (previous plan, full throttle, full brake) are always added.
2. **Simulate** all sequences in parallel with the same vehicle model (batched NumPy).
3. **Rate** each rollout with a cost function:

$$
S_k = -\text{progress} + C_{crash}\,\mathbb{1}_{crash} -
c_{under}\min(v_T, v_{ref}) + c_{over}\max(0, v_T - v_{ref}) + 
c_{close}\sum_t \max(0,\ d_{safe}(v) - d_t)^2 +
c_{steer}\sum_t \left(\frac{\Delta\delta_t}{\delta_{max}}\right)^2
$$

A rollout counts as crashed if it gets too close to the track edge **or** spins (faces more than 90° against the driving direction). After a crash its progress is frozen.

4. **Weight** the rollouts with an adaptive temperature $\lambda$ (scaled by the cost spread of non-crashed rollouts):

$$
w_k = \frac{\exp\!\big(-(S_k - S_{min})/\lambda\big)}{\sum_j \exp\!\big(-(S_j - S_{min})/\lambda\big)}
$$

5. **Update & execute:** $U \leftarrow \sum_k w_k V_k$, apply the first control, shift the plan by one step and repeat.

### 4. Hyperparameter tuning (Cross-Entropy Method)

MPPI has many coupled parameters, so they are tuned automatically with the Cross-Entropy Method:

1. Sample $K$ parameter sets from a Gaussian (scale-like parameters in log space).
2. Drive a full lap with each set on several random seeds (in parallel).
3. Score each lap: lap time if finished, otherwise $200 + \text{remaining distance}/33$, so going further is always better than crashing early.
4. Fit the new mean and standard deviation to the $k$ best sets (with a minimum spread) and repeat.

Tuned parameters: temperature factor, steering noise scale, acceleration noise, terminal speed weights $c_{under}$/$c_{over}$, speed profile grip factor, horizon length, edge penalty $c_{close}$ and steering smoothness $c_{steer}$.

## Fighting the Optimizer: Exploits & Fixes

An optimizer doesn't know what "realistic" means. It only knows the equations. Every weakness in the vehicle model was found and exploited by MPPI and CEM, and every fix made the physics more correct. These were the most interesting ones:

| # | What the car did | Why the model allowed it | Fix |
|---|---|---|---|
| 1 | **Magic brake:** deliberately spun at 42 m/s before a hairpin and lost 35 m/s in 0.24 s (~15 g) | The kinematic/dynamic blend used $v_x$ (speed along the car's axis). Sliding sideways, $v_x$ was small, so the kinematic model took over, which pulled $v_y$ to zero without any friction limit | Blend on total speed $\sqrt{v_x^2+v_y^2}$, and limit the kinematic lateral acceleration to $\mu g$ |
| 2 | **Spiral turns:** spun the car to change direction without lateral grip | A clamp $v_x \ge 0$ (meant to stop the car from rolling backwards when braking) deleted the backwards velocity of a spinning car every step, dragging the velocity vector along with the car's rotation | Brakes now act like friction (always against the motion, stopping at zero); the clamp was removed |
| 3 | **Free energy when rolling backwards:** steering while rolling backwards accelerated the car | The front slip angle assumed forward rolling, so the steering effect had the wrong sign backwards | Front slip angle computed in the wheel frame, using $\lvert v_{along} \rvert$ |
| 4 | **Ice slide:** slid sideways at 13 m/s with full brakes without losing any speed | Braking force flipped sign with $v_x \approx 0$ and cancelled out, but still used up the entire grip budget of the friction circle, so no lateral force was left | Braking force scales with the longitudinal share of the motion $v_x / \lvert v \rvert$; load transfer uses the effective braking force |
| 5 | **Reverse to the finish line:** spun at 50 m/s and rolled backwards through the last corners, still counting as a finished lap | Neither the cost function nor the lap logic penalized facing the wrong way | Facing more than 90° against the driving direction now counts as a crash, both in the MPPI cost and in the lap evaluation |

Each fix was verified with a physics test (`pytest`), e.g. a car sliding sideways with full brakes must decelerate at roughly $\mu g$ instead of keeping its speed.

**Lesson learned:** a fast lap time means nothing if the model can be exploited. Before trusting a result, check *how* the controller achieved it.

## Project Structure

```
Bicycle-Model-Simulation/
├── racesim/                    # core library
│   ├── vehicle.py              # dynamic single-track model (tires, friction circle, steering actuator)
│   ├── track.py                # track geometry, localization, crash/spin detection, speed profile
│   ├── simulate_lap.py         # shared lap loop used by run_lap and tuning
│   ├── constants.py            # physical constants
│   ├── controllers/
│   │   ├── mppi_controller.py  # MPPI controller
│   │   ├── basic_drive.py      # baseline: Pure Pursuit + PID
│   │   ├── pure_pursuit.py
│   │   └── pid_speed.py
│   ├── tuning/
│   │   └── cem.py              # Cross-Entropy Method hyperparameter search
│   └── visualization/
│       ├── animate.py          # pygame replay viewer
│       └── plot.py             # matplotlib speed plot
├── run_lap.py                  # drive one lap and save it
├── tuning.py                   # start a CEM tuning run
├── replay.py                   # replay a saved lap
├── configs/                    # MPPI parameter sets (JSON)
├── data/racetracks/            # track data (TUMFTM racetrack database)
├── results/                    # tuning logs (CSV)
├── runs/                       # saved laps (.npz, not tracked by git)
├── assets/                     # car sprite
├── docs/                       # roadmap
└── test/                       # pytest physics tests
```

## Installation

Requires Python 3.14 (tested on macOS).

```bash
git clone https://github.com/FelixxStein/Bicycle-Model-Simulation.git
cd Bicycle-Model-Simulation
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

All commands are run from the repository root.

**Drive one lap**

```bash
python run_lap.py                                  # MPPI on Hockenheim, seed 1900
python run_lap.py --controller basic_drive         # baseline controller
python run_lap.py --track Spielberg --seed 7 --log --plot
```

| Argument | Default | Description |
|---|---|---|
| `--controller` | `mppi` | `mppi` or `basic_drive` |
| `--track` | `Hockenheim` | track name (file in `data/racetracks/`) |
| `--configs` | `best_configs` | MPPI parameter set from `configs/` |
| `--seed` | `1900` | random seed of the MPPI sampler |
| `--log` | off | store MPPI rollouts for the replay viewer |
| `--plot` | off | show the speed plot after the lap |

The lap is saved to `runs/<controller>_<seed>_<track>.npz`.

**Replay a lap**

```bash
python replay.py --path runs/mppi_1900_Hockenheim.npz
```

Options: `--window_width`, `--window_height`. Press `R` to restart, `Esc` to quit.

**Tune the MPPI parameters**

```bash
python tuning.py --processes 6 --population_size 20 --seed 25 1
```

| Argument | Default | Description |
|---|---|---|
| `--population_size` | `20` | parameter sets per generation |
| `--elites` | `5` | best sets used to update the distribution |
| `--seed` | `25 1` | seeds every set is evaluated on |
| `--processes` | `6` | parallel worker processes |
| `--generations` | `50` | number of generations |

Every lap is logged to `results/<timestamp>.csv`.

## Tests

```bash
python -m pytest
```

The physics tests cover standstill braking, rolling backwards, sideways sliding, braking from high speed and consistency between single and batched simulation.

## Limitations & Next Steps

### Known limitations
- **Corners are found late:** steering noise scales with $1/v^2$, so at high speed MPPI barely explores large steering angles. It brakes hard until the noise is large enough to "discover" the corner, then turns in late.
- **Baseline fails on Spielberg:** Pure Pursuit steers towards a lookahead point on the centerline, so in tight hairpins it cuts the inside of the corner and leaves the track. It finishes Hockenheim, but not Spielberg.
- **No power limit:** full acceleration is available at any speed.
- **Simplified tires:** almost no grip loss after the peak slip angle, no tire temperature or wear.
- **Optimistic speed profile:** ignores drag and engine power on the acceleration side.
- **Results are platform-dependent:** the simulation is chaotic, so the same seed can produce different laps on different NumPy versions or CPUs.

### Small improvements
- Baseline: shorter or curvature-dependent lookahead distance so Pure Pursuit stops cutting tight hairpins
- Store the end reason (finished / off track / stuck / timeout / spin) in the saved `.npz` and use readable names instead of codes 1–5
- Make the warm start a CLI flag and load the start parameters from a JSON config instead of hard-coding them
- Use `--tuning_run` in the CSV file name, add a `--track` option for tuning
- Save the best parameter set to `configs/` automatically; print finish rate and best lap time per generation
- Pass parameter sets by name instead of by index (`params[0]` … `params[8]`)
- MPPI: compute the steering noise from the passed state instead of the real car, use a seeded RNG for rollout logging, stop growing unused log lists
- Replace magic numbers (crash score, stuck detection, sprite scaling, replay speed) with named constants
- Naming: `max_break` → `max_brake`, `get_current_steering_rate` returns an angle
- Baseline: use the track's speed profile instead of its own
- Return the actual steering angle from `simulate_lap` (it's already collected)
- Cleanup: remove the empty `animate/` folder and the tracked `.DS_Store`, add a license file
- More tests: track progress across the finish line, spin detection, an MPPI smoke test
- Viewer: draw the front wheels with the real steering angle, add a HUD (speed, lap time)

### Bigger steps
- **Steering feedforward:** center the sampling on a plan that already follows the track curvature (or a Pure Pursuit rollout as a candidate plan), so corners are found early
- **More realistic physics:** power limit, tire grip falloff, a speed profile including drag and power
- **Better terminal cost:** estimated time to the finish line instead of only the terminal speed
- **Benchmark against the time-optimal raceline** (TUMFTM `global_racetrajectory_optimization`) to separate model limits from controller limits
- **Robustness:** evaluate on many seeds and unseen tracks, multi-track tuning
- **GPU rollouts with PyTorch:** more samples and longer horizons
- **Reinforcement learning:** Gymnasium environment, PPO, comparison against MPPI

## Acknowledgements

Track data from the [TUMFTM racetrack database](https://github.com/TUMFTM/racetrack-database) (Institute of Automotive Technology, Technical University of Munich).
