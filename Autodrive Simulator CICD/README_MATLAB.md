
# Autonomous Vehicle Controller Simulation

## Overview

This project simulates an **autonomous vehicle controller** using **MATLAB** and **Simulink**. It leverages a **Clothoid Toolbox** for smooth path generation and evaluates vehicle performance across a range of real-world and synthetic tracks, including:

* VIR
* Pikes Peak
* Figure-8 maneuvers

The goal is to assess tracking accuracy, comfort, and stability under realistic driving constraints.

---

## Project Structure

```text
├── main.m                 # Primary execution script
├── VC_v1p2.slx            # Core Simulink vehicle controller model
├── init_par.m             # Vehicle dynamics and controller initialization
├── repack_simdata.m       # Post-processing of Simulink output
├── ../clothoid_toolbox    # Path generation utilities and waypoint datasets
└── results/               # (Optional) Saved plots and simulation outputs
```

---

## Getting Started

### Prerequisites

* MATLAB (R2021b or later recommended)
* Simulink
* `clothoid_toolbox` directory located in the **parent folder** of this project

---

## Running the Simulation

1. Open MATLAB and navigate to the project directory.
2. In `main.m`, specify the desired track by setting the `TESTS` array:

   ```matlab
   TESTS = [13];  % Example: VTTI track
   ```
3. Enable plot saving if desired:

   ```matlab
   saveplots = true;
   ```
4. Run the script:

   ```matlab
   main
   ```

Simulation results will be displayed and optionally saved to the `results/` directory.

---

## Configuration

### Reference Velocity & Constraints

The simulation computes a reference velocity profile subject to the following constraints:

* `vmax` – Maximum allowable speed
* `max_alat` – Maximum lateral acceleration
* `max_along` – Maximum longitudinal acceleration

These limits ensure realistic vehicle behavior and passenger comfort.

---

### Waypoint Datasets

Tracks are defined using waypoint datasets provided by the **Clothoid Toolbox**, enabling smooth curvature transitions and continuous heading profiles.

---

## Performance Validation

After each simulation run, performance is evaluated against predefined safety and comfort thresholds:

* **Cross-Track Error**
* **Velocity Error**
* **Comfort Limit (Jerk)**
* **Stability Limit (Lateral Acceleration)**
* **Security (Jamming Alert)**

Violations are flagged and visualized in post-processing.

---

## Output

* **`simdata.mat`**
  Contains all logged simulation signals, including:

  * Position
  * Heading
  * Velocity
  * Acceleration
  * Control inputs

* **Results Folder**
  If `saveplots = true`, results are saved to:

  ```text
  ./results/testcaseX/
  ```
  <p align="center">
  <img src="code/image.png" width="700">
</p>

  <p align="center">
  <img src="code/image2.png" width="700">
</p>


* **Inference Plots**
  Visualizations highlighting robustness metrics and threshold violations.

