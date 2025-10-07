# UR16e Robot Dynamics

This repository contains the final course project for *Robotics (Spring 2025)*, completed by  
**Yixiao Liu**, **Yusen Wu**, and **Huaqian Zhao**.

> ⚠️ **Note:** This repository is no longer actively maintained.  
> The code is archived for academic reference only.

---

## 📘 Overview

This project focuses on modeling and simulating the **forward and inverse dynamics** of the UR16e 6-DOF industrial manipulator.  
All computations and visualizations were implemented in MATLAB.

<div align="center">
  <img src="assets/rigid_body.png" alt="UR16e simulation" width="420"/>
  <br/>
  <em>UR16e manipulator simulation (MATLAB output).</em>
</div>

---

## 🧠 Learning Objectives

- Understand rigid-body dynamics of articulated robotic systems  
- Apply the Euler–Lagrange formulation to multi-link manipulators  
- Implement forward and inverse dynamics algorithms  
- Visualize simulation results such as torque, velocity, and trajectory profiles  

---

## 🧩 File Structure
Project organization overview:

- **src/** — main source code folder  
  - `main.m` — entry script for simulation  
  - `forward_dynamics.m` — forward dynamics computation  
  - `inverse_dynamics.m` — inverse dynamics solver  
  - `plot_results.m` — visualization of simulation results  
  - **utils/** — helper functions  
    - `dh_transform.m` — compute homogeneous transforms  
    - `compute_jacobian.m` — calculate Jacobians  
    - `inertia_matrix.m` — build inertia matrix  
- **assets/** — figures and visualization images  
  - `rigid_body.png` — main simulation visualization
