# Contact solvers

[Collision detection](model-construction.md#collision-detection) supplies
contact features with surface positions, normals, and signed gaps. These
features are passed to the contact solver as contact constraints while it
computes motion under applied forces, accounting for inertia, frictional contact,
joint limits, and joint friction.

## Formulation

CRISP provides two augmented-Lagrangian methods for the velocity-level
multi-contact nonlinear complementarity problem. Both methods enforce the
contact conditions without relaxation. Under unilateral normal contact and
Coulomb friction, each contact may separate, stick, or slide.

For more details, see our [TRO 2025 paper, Sections IV–VI](https://doi.org/10.1109/TRO.2025.3577410).

## CANAL

**Cascaded Newton-based Augmented Lagrangian** combines outer multiplier updates
with inner Newton iterations over a convex surrogate problem. It targets
accurate and robust resolution of dense contacts.

## SubADMM

**Subsystem-based Alternating Direction Method of Multipliers** uses
subsystem-based variable splitting to separate dynamics and constraint updates.
Each iteration alternates velocity solves with local updates for frictional
contact, joint limits, and joint friction.

SubADMM is a parallelizable algorithm, although CRISP does not currently exploit
this parallelism. Our [ICRA 2025 paper](https://doi.org/10.1109/ICRA55743.2025.11128665)
demonstrates how GPU parallelism enables large-scale interactive simulation.

## Numerical settings

Set the solver on the builder before constructing the model:

```cpp
builder->opt().sol.type = crisp::solver_e::canal;
builder->opt().sol.max_iter = 20;
```

Use `crisp::solver_e::sub_admm` to select SubADMM. The table covers the main
solver controls; solver-specific penalty settings are declared in
`crisp/model.hpp`.

| Option | Meaning |
| --- | --- |
| `sol.type` | Contact solver: `canal` or `sub_admm`. |
| `sol.max_iter` | Maximum outer solver iterations per time step. |
| `sol.warmstart` | Reuse solver state from the preceding time step. |

An iteration limit bounds work but does not guarantee a small residual.
SubADMM can reduce solve time, though it may leave larger residuals than CANAL
within a limited computation budget. Models with many degrees of freedom or
contacts can still exceed real-time budgets with either solver.

To change options after construction, use
[`apply_option`](api-reference.md#stepping-and-initialization) while the simulation is
stopped or locked. See [Examples](examples.md#simulation-settings) for the
solver settings.
