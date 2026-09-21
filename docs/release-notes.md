# Release notes

## 1.1.1

CRISP 1.1.1 corrects contact-stiffness handling and improves input validation in
the public builder API.

### Contact stiffness

- Geometry stiffness `k` is now handled consistently in N/m.
- Updated the assembly examples for the corrected stiffness behavior.

### Validation

- `Geometry::createConvex`, `createMesh`, `createSDF`, `createDSF`, and
  `createTDSF` now return `bool` instead of `void`.
- `register_package_root` and `set_log_level` now return `bool` instead of
  `void`.
- `ModelBuilder::build()` returns an empty handle when model construction fails.
- Invalid URDF imports return `false`.

See [Update](installation.md#update) for the package refresh procedure.

## 1.1.0

CRISP 1.1.0 adds collision pairs, solver-specific iteration limits, runtime
diagnostics, and a versioned model file format.

### Collision and model construction

- Added collision handling for sphere–SDF, capsule–SDF, and convex–SDF pairs.
- Added `mesh_t::cvx_fallback` for using a mesh's convex hull with applicable
  collision pairs. It is enabled by default.
- Separated main-stack and collision-worker workspace sizing.
  `cap().narena` sets each worker's capacity.

### Solver configuration

- Revised the default time step and solver iteration budgets.
- Added separate iteration limits for CANAL and SubADMM:
  `canal.max_iter` and `sub_admm.max_iter`.

### Viewer and diagnostics

- Added Diagnostics views for simulation-stage timings, solver iteration and
  residual histories, and stack and per-worker workspace use.
- Improved navigation, filtering, and error reporting in the Inspector and Log
  panels, and added UI scaling and default-layout restoration.
- Revised frame pacing and event handling to keep the viewer responsive during
  slow simulation steps.
- Improved transparent geometry rendering and camera sensor capture.

### Model I/O

- Added a format identifier to model files. `load_model` now rejects
  files with an incompatible or unversioned format.

### Breaking changes

- `sol.max_iter` was removed; use `canal.max_iter` or `sub_admm.max_iter`.
- `save_model` now returns `bool` instead of `void`.

See [Update](installation.md#update) for the package refresh
procedure.

## 1.0.0

Initial public release of CRISP, including prebuilt C++ packages, model
construction and simulation APIs, the interactive viewer, CANAL and SubADMM
contact solvers, and the peg-insertion and gear-assembly examples.
