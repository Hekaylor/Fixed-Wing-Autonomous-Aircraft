# Fixed-Wing Autonomous Aircraft

This repository contains models, simulations, and controllers developed for a fixed-wing autonomous aircraft system. It includes system identification, state-space modeling, LQR control, waypoint following, and trajectory tracking using MATLAB/Simulink.

## Contact Information

For any comment, questions, or concerns, please contact Hailey Kaylor at HKaylor@Mines.edu.

## Repository Structure

```
/Fixed-Wing-Autonomous-Aircraft
├── Code/                  # MATLAB scripts for linearization, control design, and plotting
├── Simulink Model/        # Simulink files for the full aircraft simulation
├── Results/               # Plots, figures, and simulation output data
└── References/            # Technical references, aircraft parameters, and documentation
```

## Overview

The project focuses on autonomous control of a fixed-wing aircraft using:

- Full 6-DOF nonlinear simulation in Simulink
- Linearized state-space models for controller design
- LQR control for longitudinal and lateral/directional dynamics
- Waypoint-following and reference tracking algorithms
- Visualization and analysis of flight path, control inputs, and tracking errors

## Getting Started

### Requirements

- MATLAB R2022b or later
- Simulink
- Aerospace Blockset (for 6-DOF modeling)

### Instructions

1. Clone the repository:

   ```bash
   git clone https://github.com/Hekaylor/Fixed-Wing-Autonomous-Aircraft.git
   cd Fixed-Wing-Autonomous-Aircraft
   ```

2. Open MATLAB and navigate to the folder.

3. To run the simulation:
   - Open the `.slx` file in `Simulink Model/`
   - Run simulations using defined input signals or controllers from `Code/`

4. Use plotting scripts in `Code/` to visualize results, or view pre-generated plots in `Results/`.

---

## Highlights

- **6-DOF nonlinear aircraft dynamics** using standard aerospace equations  
- **State-space modeling** for longitudinal and lateral/directional modes  
- **LQR controller design** with Q/R matrix tuning  
- **Waypoint guidance logic** and heading error analysis  
- **Simulink integration** with MATLAB-based post-processing  

---

## Folder Details

### `Code/`
Contains:
- Linearization scripts
- State-space matrix generation (`A`, `B`, `C`, `D`)
- LQR gain calculation
- Plotting functions for system responses and trajectories

### `Simulink Model/`
Contains:
- Full aircraft model (`6DOF_Aircraft.slx`)
- Subsystems for forces, moments, and dynamics
- Control surface input logic
- Sensor and actuator modeling

### `Results/`
Contains:
- Simulation output files
- Saved plots (e.g. altitude tracking, heading error, control signals)
- Figures demonstrating system behavior under different controller settings

### `References/`
Contains:
- Technical references and notes
- Parameter documentation
- Aircraft data sheets or reference papers used for modeling
