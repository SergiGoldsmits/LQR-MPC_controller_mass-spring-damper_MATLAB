# LQR & MPC Control Design — 6-Mass Spring-Damper System

**Course project — Industrial Controls (Controllo Industriale), MSc Industrial Automation Engineering**  
**Università degli Studi di Pavia, A.Y. 2024/2025**

---

## Overview

This project implements and compares multiple state-feedback control strategies for a **6-mass spring-damper chain** — a benchmark system in control theory representing distributed mechanical systems. The goal is to regulate the position of mass 1 to a reference setpoint by applying a force to mass 6, despite the coupling between all masses.

Controllers implemented: **continuous-time LQR**, **discrete-time LQR**, **unconstrained MPC**, **input-constrained MPC**, **state-constrained MPC**, **MPC with Kalman state estimator**, and **velocity-form MPC** for offset-free tracking.

---

## System Description

A chain of 6 identical masses connected by springs and dampers. Force is applied to the last mass (mass 6) and the position of mass 1 is the controlled output.

| Parameter | Value |
|-----------|-------|
| Number of masses | 6 |
| Mass (each) | 1 kg |
| Spring constant (k) | 10 N/m |
| Damping coefficient (h) | 2 N·s/m |
| State dimension | 12 (positions + velocities) |
| Input | Force on mass 6 [N] |
| Output | Position of mass 1 [m] |
| Sampling time (Ts) | 0.1 s |

The system is formulated in state-space and discretised using zero-order hold (ZOH) for digital control implementation.

---

## Controllers Implemented

### 1. Continuous-Time LQR (`LQR_continous.slx`)
State-feedback gain computed via Riccati equation with Q and R weighting matrices tuned to balance regulation speed against control effort.

### 2. Discrete-Time LQR (`LQR_discrete.slx`)
Discrete LQR gain (DLQR) computed from the ZOH-discretised system. Closed-loop stability verified by checking eigenvalues of (A - BK) remain inside the unit circle.

### 3. Unconstrained MPC (`MPC_unconstrained.slx`)
Receding horizon optimal control with prediction horizon N, implemented via quadratic programming (QP). No state or input constraints applied — equivalent to LQR with finite horizon.

### 4. Input-Constrained MPC (`MPC_point7.slx`)
MPC with hard constraints on the control input: u ∈ [-15, 15] N. QP solved at each timestep using MATLAB's `quadprog` with interior-point-convex algorithm.

### 5. State-Constrained MPC (`MPC_states_constrained.slx`, `MPC_constrained.slx`)
MPC with both input and state constraints enforced. State bounds: position of each mass limited to ±10 m. Inequality constraints derived from the prediction model and passed to `quadprog`.

### 6. MPC with Kalman Filter (`MPC_Kalman.slx`)
Full-state estimator (Kalman filter) added to the MPC loop — only the output (position of mass 1) is measured, and all 12 states are estimated. Handles process noise and measurement noise with tunable covariance matrices Q_K and R_K.

### 7. Velocity-Form MPC (`mympc_velocityform.m`)
Augmented state-space formulation including an integrator to achieve **offset-free tracking** in the presence of constant disturbances or model mismatch. The augmented system tracks both state and output reference simultaneously.

---

## Key Implementation Details

**Custom MPC functions (from scratch — no MPC Toolbox):**
- `mympc_project.m` — constrained MPC with input limits, solves QP via `quadprog`
- `mympc_project_constraints.m` — MPC with both input and state constraints
- `mympc_velocityform.m` — velocity-form MPC for offset-free tracking

All MPC controllers build the prediction matrices (Asig, Bsig), cost matrices (H, F), and constraint matrices analytically from the system matrices at each timestep — the full MPC formulation is implemented explicitly without toolbox abstractions.

---

## Repository Structure

```
├── main.m                          # System definition, LQR design, MPC setup
├── mympc_project.m                 # Custom MPC — input constraints
├── mympc_project_constraints.m     # Custom MPC — input + state constraints
├── mympc_velocityform.m            # Velocity-form MPC
├── point7.m                        # Velocity-form augmented system setup
├── LQR_continous.slx               # Simulink — continuous LQR
├── LQR_discrete.slx                # Simulink — discrete LQR
├── MPC_unconstrained.slx           # Simulink — unconstrained MPC
├── MPC_point7.slx                  # Simulink — input-constrained MPC
├── MPC_states_constrained.slx      # Simulink — state-constrained MPC
├── MPC_constrained.slx             # Simulink — full constrained MPC
├── MPC_Kalman.slx                  # Simulink — MPC + Kalman estimator
└── docs/
    ├── IC_Faiola_Goldsmits_Scerbo.pdf   # Project presentation
    └── mass-spring-damper-testo.pdf      # Problem statement
```

---

## How to Run

**Requirements:** MATLAB R2023a or later, Simulink, Control System Toolbox, Optimization Toolbox

1. Run `main.m` first — this defines all system matrices, computes LQR gains, equilibrium points, and sets up MPC parameters
2. Open any `.slx` Simulink model — it reads workspace variables from `main.m`
3. Run the simulation and observe step responses, control effort, and constraint satisfaction

**Recommended sequence:**
1. `LQR_continous.slx` → baseline continuous control
2. `LQR_discrete.slx` → digital implementation comparison
3. `MPC_unconstrained.slx` → MPC without constraints
4. `MPC_point7.slx` → effect of input saturation
5. `MPC_states_constrained.slx` → effect of state limits
6. `MPC_Kalman.slx` → observer-based control with partial state measurement

---

## Context

Completed as part of the **Industrial Controls** course at the University of Pavia (MSc Industrial Automation Engineering). The project demonstrates the full design and comparison workflow for modern state-feedback control — from system modelling and LQR design through to constrained MPC with state estimation — implemented from first principles without high-level toolbox abstractions.

**Authors:** Faiola · Goldsmits Ybarra · Scerbo  
**Contact:** sergigoldsmits2000@gmail.com  
**LinkedIn:** [linkedin.com/in/sergigoldsmits00](https://linkedin.com/in/sergigoldsmits00)  
**GitHub:** [github.com/SergiGoldsmits](https://github.com/SergiGoldsmits)
