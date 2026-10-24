# Source and execution audit — 2026-10-24

Historical initial audit. The later [executed dynamics report](DYNAMICS_REPORT.md)
supersedes the execution-status and repair-status statements below. Replaced
MATLAB dynamics/import sources are preserved in baselines/; the original URDF,
models, IK and planning sources remain intact. New dynamic results are URDF-based
PyBullet execution, while MATLAB/Simulink remains unexecuted.

Baseline: `da7a51b`; all original MATLAB, live scripts, Simulink models, MAT
data, meshes and helper files are preserved. ORIGINAL_INVENTORY.json records
586 original files, hashes, live-script/model XML members inspected and MAT
variable metadata. Generated cache files already in history are not evidence
of successful execution. New ignores prevent additional caches being staged.

## What exists

| Area | Observed implementation | Status |
|---|---|---|
| `mykinematics`, duplicate `RRT/myfkine`, `myikine` | Symbolic modified DH transform `Rx(alpha) Tx(a) Rz(theta) Tz(d)`; q2−pi/2 and q4+pi/2 offsets; analytic eight-branch IK | Static inspection only. Different from standard-DH chain; no verified FK/IK round trip. Unreachable/singular branches use Inf, acos not consistently clamped, inverse matrix used. |
| `mydynamics` | Standard DH transforms; COM and flange geometric Jacobians; mass matrix, gravity and proposed Coriolis; forward/inverse dynamics | Static inspection only; not validated torque dynamics. |
| `RRT` | SerialLink standard DH, joint-space RRT, discrete edge/link sampling for sphere/box obstacles, shortcut smoothing, interpolation and B-splines | Existing coursework, not verified collision avoidance. `main.m` uses millimetres and loads saved `path.mat`; `main_cartesian.m` uses UR5 dimensions. Four-DOF legacy comments/helper allocation conflict with six-DOF calls. |
| `control` | q/dq/ddq/time/tau MAT arrays, two live scripts, controller.slx; D, Q, rou robust-control parameters, trajectory derivative preparation, embedded dynamics/controller functions | Static XML and MAT inspection only; no tracking accuracy claim. Arrays have differing derivative lengths. |
| `simulink` | UR16e.slx, importrobot script, URDF and UR meshes | Static inspection only. URDF package mesh paths require resolution, cannot assume current working directory is sufficient. |
| Original results | rigid_body.png and saved arrays | Existing artifacts, not newly reproduced simulation. |
| `portfolio` (new) | Explicit SI standard-DH FK/Jacobian, smooth joint-space example, CSV/GIF export, finite-difference checks | MATLAB entry point prepared; independent Python numerical reference executed. |

## Substantive problems and scope decision

1. Convention mismatch is the main kinematic integration error: `myfkine`
   uses modified DH and hard-coded joint offsets, while `RRT/main.m` creates
   standard-DH Links and `mydynamics/dh_transform.m` is standard DH. These
   functions cannot share an unmodified parameter table or joint interpretation.
   The new isolated `ur16e_fk` implements the existing standard-DH geometry
   explicitly, avoids Symbolic Toolbox and function-name shadowing, and uses
   metres/radians with no offsets. It does not silently alter the old IK.
2. `compute_coriolis(q,q_dot,masses,I_links)` is called with masses/q_dot
   swapped. It initializes a 6x6 matrix but adds 6x1 torque vectors through
   implicit expansion, then callers multiply by q_dot again. This is not a
   verified Christoffel C matrix. Link-inertia frame handling is inconsistent.
3. `compute_gravity` projects downward physical gravity but callers treat it
   as the left-hand-side compensation term. Forward and inverse dynamics also
   use inconsistent external-wrench signs. Inertias differ from the current
   manufacturer table. These legacy routines are retained as an explicitly
   unvalidated baseline, not partially patched and advertised as correct.
4. `FindCollisionFreePath` allocates Nx4 and unconditionally sets path_found;
   sampled collision checks and smoothed paths do not certify continuous
   clearance or self-collision. RRT also needs zero-distance extension handling.
   This work does not claim to repair or certify the planner.

The focused demo therefore establishes nominal kinematics and a visualizable
joint path, not IK, torque control, collision avoidance or dynamics validation.

## Parameters and frames

[Universal Robots DH table](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics)
was checked 2026-10-24. UR16e nominal standard-DH values match the original
`mydynamics` table: a=[0,−.4784,−.36,0,0,0] m;
d=[.1807,0,0,.17415,.11985,.11655] m;
alpha=[pi/2,0,0,pi/2,−pi/2,0]. Joint angles in radians; right-handed base
coordinates; pose is frame 6/flange, without TCP or factory calibration.
The animation connects DH frame origins, not CAD surfaces or collision bodies.
Jacobian rows are base-frame linear velocity then angular velocity. Singular
values mix linear/angular units and are diagnostic, not a unit-independent
singularity margin. No exhaustive singularity analysis existed or is claimed.

## Attribution

Original README identifies a Robotics Spring 2025 team project by Yixiao Liu,
Yusen Wu and Huaqian Zhao. Individual line-level authorship and which planner
helpers were instructor supplied are not established by repository evidence;
do not credit all original code solely to Yixiao. SerialLink/Link are Peter
Corke Robotics Toolbox APIs; URDF comments identify xacro/UR description origins.
The repository license is preserved; it does not establish licenses for every
bundled third-party mesh/framework. New portfolio files and report are explicitly
new extensions; original coursework is preserved.

## Execution and reproduction boundary

MATLAB was not found on PATH or in `C:/Program Files/MATLAB`. The initial WSL
inspection was denied by the sandbox; an approved retry found neither MATLAB
nor Octave on WSL PATH, or a MATLAB installation under /usr/local or /opt.
No MATLAB release/toolbox license could be
verified. Model metadata identifies **R2025a**, with an older controller backup
named r2023b. Use R2025a or newer for the original models; exact toolbox/runtime
compatibility still requires a licensed run. Legacy code needs Symbolic Math
Toolbox, Peter Corke Robotics Toolbox, Robotics System Toolbox (`importrobot`),
Simulink and model-dependent Simscape/Multibody support; knnsearch also needs
Statistics and Machine Learning Toolbox if that unused helper is invoked.

The new demo targets base MATLAB R2023b+ (`writematrix`, `jsonencode`, graphics,
getframe/imwrite), but has **not been executed in MATLAB** on this host.
Run from the root:

```matlab
addpath('portfolio');
run_demo('outputs/matlab_demo_01'); % choose a fresh output directory
```

Independently executed reference (Python 3.12, NumPy/SciPy/Matplotlib/Pillow):

```powershell
python portfolio/verify_reference.py
```

It computes the same nominal transform equations, verifies 300 Jacobian
columns against central differences of translation and rotation with h=1e-6,
and renders actual computed joint configurations. Seed 5170; 50 configurations;
maximum residual 1.30e−10. This is mathematical evidence for the equations,
**not execution validation of the MATLAB files or Simulink models**. The new
6 s quintic joint interpolation has 121 samples and sampled flange-path length
0.75117 m. No end-effector target tracking or dynamic feasibility is implied.
JSON, CSV and plots are in results/. MATLAB uses its own RNG, so random
configurations need not match Python despite the same seed.
