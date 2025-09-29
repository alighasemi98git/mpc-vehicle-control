# Model Predictive Control for Vehicle Dynamics (EL2700)

This repository contains the implementation of **state feedback control design** for a vehicle model, as part of the *Model Predictive Control (EL2700)* course at KTH. The project focuses on simulating and controlling the lateral dynamics of a simplified kinematic bicycle model to achieve safe lane-change maneuvers, using **Python** and **CVXPY**.

## Project Overview

* **Course:** EL2700 – Model Predictive Control
* **Topic:** Discrete-time linear state feedback design
* **Context:** Lane-change maneuver for obstacle avoidance
* **Technologies:** Python 3, CVXPY, control library, NumPy, Matplotlib

## Task Objectives

1. **Model Derivation**

   * Discrete-time state-space model of the bicycle dynamics.
   * Verification of continuous-to-discrete transformation.

2. **System Analysis**

   * Transfer function, poles, and zeros.
   * Comparison of continuous vs discrete dynamics.

3. **State Feedback Controller**

   * Pole placement design using the `place` method.
   * Closed-loop control for safe lane-change maneuver.

4. **Performance Constraints**

   * Steering rate |u| < 0.25
   * Lateral velocity |vy| < 1 m/s
   * No overshoot beyond 1 m offroad
   * Reach 75% of reference within 10 s

5. **Integral Action**

   * Compensation for constant lateral velocity disturbance (0.02 m/s).
   * Extended controller with integral term for robust tracking.

## Repository Structure

```
.
├── src/
│   ├── linear_car_model.py    # Vehicle dynamics and system matrices
│   ├── controller.py          # Feedback + integral control implementation
│   ├── task1.py               # Main entry point for running simulations
│
├── report/
│   └── Assignment1_Report.pdf # Full academic report with derivations
│
├── requirements.txt           # Python dependencies
└── README.md
```

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/<your-username>/mpc-vehicle-control.git
   cd mpc-vehicle-control
   ```

2. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

3. (Optional) Install `slycot` for advanced control tools:

   ```bash
   sudo apt install gfortran liblapack-dev libblas-dev
   pip install slycot
   ```

## Usage

Run the main script to simulate the lane-change maneuver:

```bash
python src/task1.py
```

This will:

* Simulate the lateral dynamics of the vehicle.
* Apply the designed state feedback controller.
* Display trajectory plots, control signals, and constraint checks.

## Results

* **Deterministic case:** vehicle successfully changes lane while respecting input and velocity constraints.
* **With disturbance:** integral action ensures correct lane following within 30 seconds.

## Technologies

* **Python 3.10+**
* **CVXPY** – optimization framework
* **NumPy, SciPy, Matplotlib** – numerical and plotting tools
* **control, slycot** – system dynamics analysis

## Authors

* Ali Ghasemi ([aghasemi@kth.se](mailto:aghasemi@kth.se))
* Group members (if applicable)

*KTH Royal Institute of Technology – EL2700 Model Predictive Control (2025)*

---

📄 This repository combines the assignment report and Python implementation for state feedback controller design in the context of vehicle dynamics.
