# Preserved MATLAB and Simulink coursework

Python in `portfolio/` is the primary runnable implementation. These MATLAB
sources and data retain the original team attribution: Yixiao Liu, Yusen Wu
and Huaqian Zhao. They have not been executed in MATLAB during this extension.

- `mykinematics/`, `RRT/`, `control/`: retained original MATLAB/Simulink source
  and reference data, moved with Git history intact. The analytic IK and full
  robust/noise-control Simulink model have not been declared equivalent to Python.
- `mydynamics/`: previously repaired MATLAB dynamics; the parameter-file path
  now resolves the repository root from this legacy location.
- `simulink/`: preserved `.slx` model and repaired import script. The original
  URDF and meshes stay at repository-root `simulink/` as shared model assets.
- `baselines/`: byte-preserved pre-repair dynamics/import sources. Do not put
  these duplicate function names on the same MATLAB path as repaired sources.

To inspect with a licensed MATLAB installation, use `addpath('legacy/mydynamics')`
explicitly, or run `legacy/simulink/import_urdf.m`. R2025a model/toolbox requirements
are documented in the reports. Model execution may need additional relative-path
setup; no claim of MATLAB numerical equivalence is made. Generated slprj/slxc
files were removed from version control, with their earlier history retained.
