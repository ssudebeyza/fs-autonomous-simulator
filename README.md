# Formula Student Autonomous Vehicle Simulator

A Python-based Formula Student autonomous vehicle simulation developed to explore path planning, vehicle control, perception, telemetry analysis, and theoretical lap-time simulation.

The project began as a Pure Pursuit path-following simulator and was progressively expanded to include simulated camera perception, cone-based local path generation, Model Predictive Control (MPC), curvature-based speed planning, telemetry analysis, and a Quasi-Steady-State (QSS) lap simulation.

> This is an educational engineering project. Vehicle, aerodynamic, tyre, and sensor parameters are assumed/simplified unless otherwise stated and should not be interpreted as data from a real Formula Student vehicle.

---

## Project Overview

The simulator models a Formula Student-style autonomous vehicle driving around a cone-defined technical circuit.

The autonomous control pipeline is:

```text
Track Cones
     ↓
Simulated Camera
     ↓
Visible Cone Detections
     ↓
Local Path Generation
     ↓
Pure Pursuit / MPC
     ↓
Curvature Speed Planner
     ↓
Vehicle Model
     ↓
Telemetry & Performance Analysis
```

The project also contains a separate QSS model for estimating theoretical lap performance under assumed vehicle limits.

---

## Features

### Simulated Camera Perception

A synthetic front-facing camera projects track cones into a perspective camera view.

The camera model includes:

- Horizontal field of view
- Maximum detection range
- Perspective projection
- Relative cone position
- Blue/yellow cone classification
- Camera preview inside the simulator

The current camera system uses known simulator cone coordinates to generate synthetic detections. It is therefore a simulated perception model rather than a real computer-vision pipeline.

---

### Cone-Based Local Path Generation

Visible blue and yellow cones are used to construct a local driveable path.

The path generator includes:

- Cone filtering in vehicle coordinates
- Boundary handling
- Cone pairing
- Track-centre estimation
- Hairpin handling
- Path continuity checks
- Path smoothing
- Short-term path memory
- Recovery behaviour when perception becomes insufficient

The controller follows the generated local path rather than directly following the global racing line.

---

## Pure Pursuit Controller

Pure Pursuit is retained as the baseline path-following controller.

Features include:

- Dynamic lookahead distance
- Nearest-path-point search
- Steering-angle calculation
- Steering limits
- Local-path tracking

Pure Pursuit provides a useful reference controller for comparison with MPC.

---

## Model Predictive Control

The simulator also includes a finite-horizon predictive steering controller.

The MPC evaluates candidate steering sequences using a simplified vehicle model and selects the command with the lowest predicted cost.

The cost function considers:

- Path tracking error
- Heading error
- Steering magnitude
- Steering change
- Forward progress
- Cone clearance
- Terminal path error
- Terminal heading alignment

A two-stage steering sequence allows the controller to predict both corner entry/apex steering and steering unwind at corner exit. This was particularly important for negotiating the circuit's tight hairpin.

> The controller is a lightweight candidate-search MPC implementation rather than a full optimisation-based constrained MPC solver.

---

## Speed Planning

Target speed is generated from local path geometry.

The curvature-based speed planner considers:

- Local path curvature
- Lateral acceleration limit
- Maximum vehicle speed
- Acceleration limit
- Braking limit
- Preview distance

The vehicle therefore reduces speed for high-curvature sections and accelerates as the path opens.

---

## Vehicle Model

Vehicle motion is simulated using a kinematic bicycle model.

The model includes:

- Vehicle position
- Yaw angle
- Steering angle
- Wheelbase
- Longitudinal acceleration
- Braking
- Maximum speed

Simulation units are converted between screen coordinates and SI units using a defined pixels-per-metre scale.

---

## Telemetry

Telemetry is recorded during simulation at approximately 10 Hz.

Logged parameters include:

- Time
- Vehicle speed
- Target speed
- Steering angle
- X/Y position
- Yaw
- Nearest path index
- Reference cross-track error
- Local path error
- Active path source

Telemetry can be analysed after a run to evaluate controller behaviour and vehicle performance.

---

## Lap Performance

The simulator tracks completed laps and calculates performance metrics including:

- Lap time
- Average speed
- Maximum speed
- Mean cross-track error
- Cross-track-error RMSE

These metrics can be used to compare Pure Pursuit and MPC under the same simulated track conditions.

---

# Quasi-Steady-State Lap Simulation

A separate QSS module estimates theoretical lap performance using assumed vehicle limits.

The QSS calculation uses:

- Track geometry
- Track curvature
- Maximum lateral acceleration
- Longitudinal acceleration capability
- Braking capability
- Power limitation
- Aerodynamic drag
- Rolling resistance

The solver performs forward acceleration and backward braking passes to construct a theoretical speed profile around the circuit.

With the current assumed vehicle parameters, one simulation produced approximately:

```text
Track Length:          427.23 m
Theoretical Lap Time:   30.79 s
Average Speed:          13.88 m/s
Maximum Speed:          28.42 m/s
Minimum Speed:           5.68 m/s
```

These values are theoretical outputs from the assumed model and are not claimed to represent a real Formula Student vehicle.

The QSS module can also generate:

- Speed profile vs distance
- Track curvature vs distance
- Speed map around the circuit

---

## Project Structure

```text
fs-autonomous-simulator/
│
├── main.py
├── vehicle.py
├── track.py
├── controller.py
├── mpccontroller.py
├── camera.py
├── pathgen.py
├── perception.py
├── speedplanner.py
├── telemetry.py
├── plottelemetry.py
├── laptimer.py
├── performance.py
├── graphics.py
├── utils.py
│
├── qss/
│   ├── plots.py
│   ├── run.py
│   ├── solver.py
│   ├── trackmodel.py
│   └── vehiclemodel.py
│
├── requirements.txt
└── README.md
```

---

## Controls

During the interactive simulation:

```text
M     Toggle manual / autonomous mode
C     Toggle Pure Pursuit / MPC
R     Reset vehicle
ESC   Exit
```

---

## Installation

Clone the repository:

```bash
git clone https://github.com/ssudebeyza/fs-autonomous-simulator.git
cd fs-autonomous-simulator
```

Install dependencies:

```bash
pip install -r requirements.txt
```

Run the interactive simulator:

```bash
python main.py
```

Run the QSS lap simulation:

```bash
python -m qss.run
```

---

## Technologies

- Python
- Pygame
- NumPy
- Matplotlib
- Git / GitHub

---

## Engineering Scope

This project was developed as a learning platform for Formula Student autonomous systems and vehicle-performance engineering.

The main areas explored were:

- Path-following control
- Pure Pursuit
- Model Predictive Control
- Local path planning
- Synthetic sensor modelling
- Vehicle dynamics
- Curvature-based speed planning
- Controller tuning
- Telemetry analysis
- QSS lap simulation
- Engineering debugging and iterative validation

The project intentionally uses simplified models so that individual control and vehicle-performance concepts can be studied independently.

---

## Limitations and Future Development

Current limitations include:

- Synthetic rather than image-based cone detection
- Simplified kinematic vehicle dynamics
- Simplified tyre behaviour
- No combined tyre friction ellipse
- No aerodynamic downforce model in the interactive vehicle
- Idealised sensor measurements
- QSS results based on assumed vehicle parameters

Possible future extensions include:

- Real image-based cone detection
- Sensor noise and uncertainty
- Dynamic bicycle model
- Tyre friction ellipse
- Aerodynamic load model
- More advanced MPC optimisation
- Controller benchmarking across multiple tracks

---

## Author

Developed by **Sude Beyza** as an independent Formula Student autonomous vehicle simulation and vehicle-performance learning project.