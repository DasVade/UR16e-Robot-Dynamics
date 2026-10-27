# Legacy model and implementation notes

The Python workflow and measured dynamics results are documented in
[PYTHON_WORKFLOW.md](PYTHON_WORKFLOW.md) and [DYNAMICS_REPORT.md](DYNAMICS_REPORT.md).
The original MATLAB/Simulink sources are under `legacy/`; pre-repair dynamics
and import routines are in `legacy/baselines/`. Baseline `da7a51b` and
`ORIGINAL_INVENTORY.json` record 586 original files, hashes, model/live-script
XML members and MAT variable metadata.

## Original implementation

| Area | Implementation and engineering observations |
|---|---|
| `legacy/mykinematics`, `legacy/RRT/myfkine`, `myikine` | Symbolic modified DH `Rx(alpha) Tx(a) Rz(theta) Tz(d)`; q2−pi/2 and q4+pi/2 offsets; analytical eight-branch IK. Some unreachable/singular branches use Inf; acos arguments are not consistently clamped. |
| `legacy/mydynamics` | Standard DH, COM/flange geometric Jacobians, mass, gravity, velocity terms and forward/inverse dynamics. Corrected routines and pre-repair copies are retained. |
| `legacy/RRT` | Peter Corke SerialLink/Link standard DH, joint-space RRT, sampled sphere/box collision checks, shortcut smoothing and B-spline interpolation. `main.m` uses millimetres and saved `path.mat`; `main_cartesian.m` uses UR5 dimensions. |
| `legacy/control` | q/dq/ddq/time/tau arrays, two live scripts, `controller.slx`, D/Q/rou robust-control parameters and embedded dynamics/controller functions. Some derivative arrays have differing lengths. |
| `legacy/simulink` | Simulink model and import script; shared URDF/UR meshes remain at root `simulink/`. Mesh package paths need resolution. |

## Corrections and integration choices

- The modified-DH kinematics and standard-DH dynamics/RRT use different joint
  interpretations. The SI standard-DH reference uses metres/radians without
  the modified-DH joint offsets. Python reconciles this chain with the URDF
  base/tool transforms; it uses bounded numerical IK.
- Original `compute_coriolis(q,q_dot,masses,I_links)` calls swap masses and
  q_dot. The implementation initializes a 6×6 matrix but adds 6×1 torque
  vectors by implicit expansion, and callers multiply by q_dot again.
  The repaired dynamics use consistent velocity-dependent torque dimensions.
- Original gravity code projects downward physical gravity while callers
  interpret it as a compensation term. Forward/inverse dynamics also use
  inconsistent external-wrench signs. The current equations and independent
  energy checks are detailed in the dynamics report, including inertia/COM frames.
- `FindCollisionFreePath` allocates N×4 for six-joint calls and sets path_found
  unconditionally. The Python planner handles six joints, zero-distance
  extensions, edge checks and post-smoothing validation. Timed reference and
  simulated tracking states are checked separately.

## Nominal DH reference

[Universal Robots DH parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics)
give a=[0,−.4784,−.36,0,0,0] m; d=[.1807,0,0,.17415,.11985,.11655] m;
alpha=[pi/2,0,0,pi/2,−pi/2,0]. These match the original standard-DH table.
The reference pose is frame 6/flange in a right-handed base, without a TCP or
factory calibration. Jacobian rows are base-frame linear then angular velocity;
singular values mix their units and serve as numerical diagnostics.

The early Python reference used seed 5170 and 50 configurations, checking 300
Jacobian columns with central differences h=1e-6. Maximum residual was 1.30e−10.
Its kinematic six-second quintic interpolation has 121 samples and sampled
flange-path length 0.75117 m. That frame-origin playback is distinct from the
later URDF-mesh torque-driven experiments.

## MATLAB requirements and source references

Model metadata identifies R2025a and an older r2023b controller backup. The
original route needs licensed MATLAB, Symbolic Math Toolbox, Peter Corke
Robotics Toolbox, Robotics System Toolbox (`importrobot`), Simulink and
model-dependent Simscape/Multibody support. `knnsearch` additionally requires
Statistics and Machine Learning Toolbox. MATLAB/Simulink was unavailable on
the experiment host; Python is the executed workflow.

SerialLink/Link are Peter Corke Robotics Toolbox APIs; URDF comments retain
xacro/UR-description references. Existing source licenses and notices remain
with their files.
