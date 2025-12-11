# 2D Planar VTOL Simulation

A Python-based simulation environment for studying **2D planar Vertical Takeoff and Landing (VTOL)** dynamics, **nonlinear state estimation**, and **control design**.  
This project supports **PID control**, a **nonlinear observer (NLO)**, and an **Extended Kalman Filter (EKF)**, with tools for visualization and experimentation.

---

## 📌 Features

### Dynamics & Simulation
- 6-state planar VTOL model  
  \[
  x = [z,\, h,\, \theta,\, \dot{z},\, \dot{h},\, \dot{\theta}]^T
  \]
- Includes mass/inertia parameters, aerodynamic drag, gravity, and rotor forces.
- Configurable process noise and simulation timestep.
- Deterministic or stochastic integration.

### Control
- `ctrlPID` implements independent PID loops for altitude, lateral position, and pitch.
- Reference generation using `signalGenerator` for step, sinusoids, or custom trajectories.

### State Estimation

Supports two observer frameworks:

#### 1. Nonlinear Observer (NLO)
- Based on Lipschitz / one-sided Lipschitz error bounds.
- Uses:
  \[
  \dot{\tilde{x}} = (A - HC)\tilde{x} + \phi(x) - \phi(\hat{x})
  \]
- Observer gain chosen so \(A - HC\) is Hurwitz.

#### 2. Extended Kalman Filter (EKF)
- Prediction using linearized Jacobian of the dynamics.
- Adaptive measurement noise \(R\) (when LiDAR or range/bearing sensors are active).
- Measurement model:
  \[
  y = Cx
  \]
  or a nonlinear LiDAR model if enabled.

### Visualization
- `VTOLAnimation` produces a real-time animation of the 2D vehicle.
- `dataPlotter` provides:
  - True vs estimated states  
  - Control inputs  
  - Estimation error and covariance (if using EKF)

---

## 📁 Project Structure

```text
.
├── VTOLDynamics.py        # Nonlinear 2D VTOL dynamics model
├── ctrlPID.py             # PID controller
├── observer.py            # Nonlinear observer AND/OR EKF implementation
├── VTOLAnimation.py       # Visualization of vehicle motion
├── dataPlotter.py         # Time-series plotting utilities
├── signalGenerator.py     # Reference trajectory generation
├── main.py                # Main simulation loop
├── utils/                 # Optional helpers (Jacobians, noise models, etc.)
└── README.md              # This file
