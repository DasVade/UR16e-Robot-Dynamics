# MATLAB-to-Python feature mapping

The primary interface is `python portfolio/cli.py`. Python uses the same supplied
URDF/meshes and established algorithms where practical; it is not an exact port
of unexecuted MATLAB or Simulink numerical behavior.

| Coursework feature | Python implementation | Status and distinction |
|---|---|---|
| URDF/mesh import | `urdf_model.prepare`, `robot.Robot` | Executed; package paths resolved, DAE visuals converted to OBJ, inertia-file loading explicit |
| Modified/standard DH FK | `Model.frames`, standard-DH audit | Executed URDF FK and reconciled standard-DH comparison; modified-DH student function retained in legacy, not silently mixed into Python |
| Geometric Jacobians | `Model.jac`, `Robot.jacobian` | Executed; engine and finite-difference checks |
| Analytical eight-branch IK / report's iterative IK | `Robot.ik` | Replacement: bounded numerical full-pose least squares; local residual checks, not exact branch enumeration or global coverage |
| M, gravity, Coriolis/velocity terms | `Robot.mass`, `gravity`, `velocity_terms`; independent `Model.dynamics` | Executed engine calculations cross-checked by independent URDF kinetic/potential-energy formulation |
| Forward/inverse dynamics and external wrench | `Robot.forward`, `inverse` | Executed, [force;moment] in base_link/world at tool0; opposite signs in forward/inverse equations; original MATLAB defects separately documented |
| Joint interpolation/B-spline timing | `robot.reference` | Replacement for integrated execution: C2 quintic timing on each validated straight edge, stopping at knots. No claim of time-optimal B-spline equivalence |
| PD+gravity / computed torque | `run_experiments`, shared engine interface for integration | Executed matched comparisons and 1 kg mismatch; original robust/noise Simulink blocks are unsupported in Python |
| Joint-space RRT and shortcut smoothing | `planner.rrt`, `CollisionScene.edge` | Executed adaptation of original ideas, with zero-distance guard, bounds, clearance margin and finer post-smoothing edge checks |
| Collision-checked planning + dynamic execution | `cli plan-track` | Newly implemented: blocked direct edge, RRT detour, timing, torque-driven tracking, independent reference/actual checks |
| Export/figures/mesh animations | `cli visualize`, rendering scripts | Executed CSV/NPZ/JSON, plots, GIF and H.264 MP4 with actual meshes and checked obstacles |
| Original Simulink robust/noise scenarios, STEP assembly, manufacturer calibration | `legacy/` / source archive | Not migrated or validated; not needed by the runnable URDF pipeline |

Seven tests cover the previous independent model checks, shared robot API,
edge-preserving timing, and independently checked integrated path/recorded
collision and limit metrics. All original algorithm/contribution boundaries
remain attributed to the group; subsequent integration and numerical checks
are extensions, not original robotics algorithms.
