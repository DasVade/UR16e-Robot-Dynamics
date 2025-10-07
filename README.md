# UR16e Robot Dynamics

This repository contains the course project for *Robotics (Spring 2025)*, completed by  
**Yixiao Liu**, **Yusen Wu**, and **Huaqian Zhao**.

> ⚠️ **Note:** This repository is no longer actively maintained.  
> The code is archived for academic reference only.

---

## 📘 Overview

This project focuses on the **forward and inverse dynamics** of a UR16e 6-DOF industrial manipulator.  

<div align="center">
  <img src="assets/rigid_body.png" alt="UR16e simulation" width="420"/>
  <br/>
  <em>UR16e manipulator simulation (MATLAB output).</em>
</div>

The implementation includes:
- Kinematic modeling based on Denavit–Hartenberg parameters  
- Symbolic dynamics derivation using the Euler–Lagrange formulation  
- Simulation of joint motion, torque profiles, and end-effector trajectories  
- Visualization and plotting of dynamic responses  

Developed primarily in **MATLAB**, with numerical solvers and plotting utilities for analysis.

---

## 🧠 Learning Objectives

- Understanding rigid-body dynamics of articulated robotic systems  
- Applying the Euler–Lagrange method to multi-link manipulators  
- Implementing forward/inverse dynamics computation  
- Gaining experience in simulation, data analysis, and visualization  

---

## 🧩 File Structure

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
src/
├── main.m
├── forward_dynamics.m
├── inverse_dynamics.m
├── plot_results.m
├── utils/
│   ├── dh_transform.m
│   ├── compute_jacobian.m
│   └── inertia_matrix.m
└── assets/
└── rigid_body.png
