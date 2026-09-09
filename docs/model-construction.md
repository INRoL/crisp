# Model construction

A CRISP model contains bodies, joints, physical properties, collision geometry,
and visual geometry. Configure it with `ModelBuilder`, then call `build()`.

## Bodies and joints

A body has a pose, mass, and inertia. `addBody(..., true)` fixes it to the world;
`addBody(..., false)` creates a freely moving body. Use `Body::addChild` to connect
a child body with a joint. Public joint types include fixed, floating, revolute,
and prismatic joints.

`ModelBuilder::addRobot` imports a URDF. Pass an absolute path or a path relative
to the working directory; mesh paths in the URDF are resolved from the URDF’s
directory. Check the return value before configuring joints or actuators.

## Collision geometry

Attach geometry with `Body::addGeom`. Choose a representation to suit the shape
and the contacts of interest:

| Representation | Use |
| --- | --- |
| Primitive | Planes, spheres, boxes, capsules, and cylinders described by dimensions. |
| Convex | Convex geometry supplied through the convex-shape interface. |
| Triangle mesh | Surface geometry from mesh files. Mesh resolution is part of the contact model. |
| Signed distance field (SDF) | Implicit geometry evaluated by a registered function and its parameters. |
| Differentiable support function (DSF) | Smooth support-function descriptions of convex geometry. |

Primitive size helpers such as `make_box_size` take full dimensions and produce
the size representation expected by `geometry_t`. For example:

```cpp
auto& geom = body.addGeom({
  .type = crisp::geometry_e::box,
  .size = crisp::make_box_size(0.1, 0.2, 0.3)
});
```

A geometry has a local pose relative to its body. Its contact settings include
friction (`mu`) and collision-filter fields (`con_type`, `con_affinity`). Set
`visual = false` for collision-only geometry. To make a visual-only geometry,
set both contact-filter fields to zero.

CRISP supports custom, parameterized SDFs. The zero level set defines the
surface, while a callback evaluates the field and its derivatives. Parameters
can encode dimensions and other shape properties, allowing one callback to
represent a family of geometries.

DSFs describe smooth convex bodies by their extent
in each direction and provide derivatives to collision detection. See the
[RSS 2023 paper](https://doi.org/10.15607/RSS.2023.XIX.043) for the
representation and the
[IROS 2024 paper](https://doi.org/10.1109/IROS58592.2024.10802286) for collision
detection via Riemannian optimization.

## Collision detection

Collision detection proceeds in two stages. A broad pass removes geometry pairs
that cannot interact, using collision filters, model exclusions, and spatial
bounds. A narrow pass then applies the algorithm for each remaining geometry
pair and produces contact features within the detection margin.

Each feature contains a point on each geometry, a contact normal, and a signed
gap: a positive gap means the surfaces are separated, while a negative gap
indicates overlap. One geometry pair can produce several features so that
extended or nonconvex contact regions are represented by more than one point.

The detected features are passed to the contact solver as contact constraints
for the current simulation step. Where supported by the geometry-pair algorithm,
contact reduction retains representative features to limit solver work. This
balances contact coverage against cost; a smaller set may omit details needed
to support the geometry.

## Custom distance functions

Register an SDF evaluation function and an axis-aligned bounding-box function
with `register_sdf`, then pass the returned type identifier to `createSDF`.
The evaluation function receives a query point in the geometry's local frame
and its parameter vector. It supplies a scalar field value and indicates which
spatial derivatives it has provided.

The function and its derivatives must describe the same geometry. An optional
axis-aligned bounding-box callback can provide bounds for collision detection; if
it is omitted, CRISP attempts to estimate the bounds automatically. Incorrect
bounds can exclude valid contacts, and an unsuitable gradient can affect contact
normals. See [Examples](examples.md) for registrations and geometry definitions,
and the [callback declarations](api-reference.md#custom-sdf-registration) for the
interface.

## Actuation

CRISP supports position and force actuators, both attached to joints. For a
position actuator, `u` specifies a position target and `udot` specifies a
velocity target. Its three gain entries represent an additive force or torque,
a position-error gain, and a velocity-error gain. A force actuator uses `u` as a
generalized force input.

Check `addActuator`'s return value and keep the actuator ordering consistent with
your input vector. The model supplies the actuator count as `size.nu`.
[Using CRISP](using-crisp.md#control-a-model) shows where these inputs enter the
simulation loop.

## Simulation options

Set simulation options through `builder->opt()` and capacities through
`builder->cap()` before calling `build()`:

```cpp
builder->opt().dt = 0.005;
builder->opt().con.erp = 0.1;
builder->cap().nthread = 1;
```

| Setting | Default | Meaning |
| --- | --- | --- |
| `opt().dt` | `0.01` | Simulation time step in seconds. |
| `opt().gravity` | `{0, 0, -9.81}` | World-frame gravitational acceleration in m/s². |
| `opt().con.margin` | `{0.01, 0.01, 0.01}` | Broad-phase absolute padding (m), relative padding, and narrow-phase detection distance (m), in that order. |
| `opt().con.erp` | `0.1` | Error reduction parameter for contact penetration correction. |
| `opt().con.cache` | `true` | Reuse collision detection information from the preceding step. |
| `cap().nthread` | `1` | Total thread count, including the calling thread. Use `0` to select the hardware thread count automatically. |
| `cap().ncon_max` | `100` | Maximum contact features retained across the model per step; additional features are discarded. |

CRISP runs simulation on the CPU, with parallelism currently limited to collision
detection. Solver cost still grows with the number of contact points, so limiting
them helps keep step times manageable. However, retaining too few can omit
contacts needed to represent an interaction and lead to unstable behavior.

See [Contact solvers](contact-solvers.md#numerical-settings) for solver settings.
To change `opt()` settings after construction, use
[`apply_option`](api-reference.md#stepping-and-initialization) while the simulation
is stopped or locked. Capacities must be set before construction.
