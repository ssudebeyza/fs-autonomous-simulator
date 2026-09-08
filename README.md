# Formula Student Path Tracking Simulator

A Python-based Formula Student vehicle control simulator featuring Pure Pursuit steering, PID speed control, racing-line generation and curvature-based speed planning.

The project was developed to explore the fundamentals of autonomous vehicle control and path tracking in a lightweight 2D simulation environment.

## Demo

A demonstration image or GIF will be added here.

## Features

- Pure Pursuit path-tracking controller
- PID longitudinal speed controller
- Curvature-based target-speed planning
- Procedural oval track generation
- Racing-line generation
- Manual and automatic driving modes
- Kinematic bicycle vehicle model
- Formula-style vehicle visualisation
- Live speed, steering and mode display
- Modular Python project structure

## Controls

| Key | Function |
|---|---|
| W | Accelerate in manual mode |
| S | Brake or reverse in manual mode |
| A | Steer left |
| D | Steer right |
| M | Switch between manual and automatic mode |
| R | Reset the vehicle |
| ESC | Close the simulator |

## Project Structure

"fs-autonomous-simulator/
├── controller.py
├── graphics.py
├── main.py
├── speedplanner.py
├── track.py
├── utils.py
├── vehicle.py
├── requirements.txt
├── .gitignore
└── README.md"
## Control Architecture

"The simulator separates the vehicle-control problem into three main components:

Pure Pursuit Controller

The Pure Pursuit controller identifies a target point ahead of the vehicle on the racing line and calculates the steering command required to approach it.

PID Speed Controller

The PID controller compares the vehicle speed with the current target speed and produces an acceleration or braking command.

Curvature-Based Speed Planner

The speed planner estimates the local curvature of the racing line. Higher speeds are requested on straights, while lower target speeds are assigned to tighter corners."

## Installation

"Clone the repository:
git clone https://github.com/ssudebeyza/fs-autonomous-simulator.git
cd fs-autonomous-simulator
Install the required package:
pip install -r requirements.txt
Run the simulator:
python main.py
Technologies

* Python
* Pygame
* Object-oriented programming
* Kinematic bicycle model
* Pure Pursuit control
* PID control"

## Current Limitations

"This project currently uses a predefined track and racing line. It does not yet include perception, cone detection, SLAM or sensor fusion.

Therefore, the current version is primarily a path-tracking and vehicle-control simulator rather than a complete autonomous-driving stack."

## Planned Development

"* Telemetry logging to CSV
* Speed and steering plots
* Lap timing
* Cross-track error calculation
* Track-boundary detection
* Virtual camera or LiDAR field of view
* Cone-based path generation
* Controller comparison
* More complex Formula Student tracks"

## Author

Developed by Sude Beyza as a personal Formula Student controls and simulation project.