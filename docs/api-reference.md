# API reference

This page summarizes the public C++ operations used in the guides: model
construction, stepping, state access, and application callbacks. Include
`<crisp/crisp.hpp>` and link `crisp::crisp`. All names below are in namespace
`crisp`. The headers shipped with the CRISP package provide the full declarations
and overloads for that release.

## Construction and ownership

```cpp
model_builder_ptr make_model_builder();
model_ptr ModelBuilder::build();

data_ptr make_data(model_t const& m);

app_ptr make_app(model_ptr m, bool run = true);
app_ptr make_app(char const* file_name, bool run = true);
app_ptr make_app(path_t const& file_name, bool run = true);
```

- `make_model_builder` creates a builder; `ModelBuilder::build` returns the
  completed model.
- `make_data` allocates simulation data for a model. Initialize it with `reset`.
- `make_app` creates an application. The `model_ptr` overload takes ownership;
  the file-name overloads load the model first.

Keep the model alive while using its data. The owning handles release their
objects when destroyed. Views returned from model/data storage do not transfer
ownership; do not retain them after their owner is destroyed.

## Model files

```cpp
using path_t = std::filesystem::path;

model_ptr load_model(char const* file_path);
model_ptr load_model(path_t const& file_path);

bool save_model(model_t const& m, char const* file_path);
bool save_model(model_t const& m, path_t const& file_path);
```

Use `save_model` to store a built model and `load_model` to restore it. Model
files are versioned and checked when loaded, but compatibility is guaranteed
only within the same CRISP version. Keep the scene definition as the source of
truth and recreate saved models after updating CRISP.

Always check the result of `load_model` before passing it to `make_data` or
`make_app`.

## Stepping and initialization

```cpp
void reset(model_t const& m, data_t& d);
void step(model_t const& m, data_t& d);
void forward(model_t const& m, data_t& d);
void apply_option(model_t& m, model_t::option_t const& opt);
```

- `reset` initializes the simulation time, generalized state, and actuator inputs from the model's initial configuration. It zeroes velocities and actuator velocity targets.
- `step` advances the simulation by one model time step and updates dynamics and contact results in `d`.
- `forward` updates position-dependent quantities from the current generalized positions. It does not advance time or solve dynamics and contacts.
- `apply_option` applies simulation options to an existing model. Synchronize access if the model is running in an application.

For feedback control that requires refreshed kinematic quantities before
actuation, the step is also exposed as:

```cpp
void step1(model_t const& m, data_t& d);
void step2(model_t const& m, data_t& d);
```

Call `step1`, provide inputs, then call `step2` once to complete the step.
`step1` advances the time counter and prepares position/velocity quantities;
`step2` applies actuation, resolves contact and dynamics, and integrates the
state. Do not additionally call `step` for that same step. `AppManager` places
its control callback between these two operations.

## State and inputs

| Member or view | Meaning |
| --- | --- |
| `model.size.nq`, `model.size.nv`, `model.size.nu` | Generalized-position, generalized-velocity, and actuator-input dimensions. |
| `data.state.q()`, `data.state.v()` | Generalized positions and velocities. |
| `data.act.u()`, `data.act.udot()` | Actuator inputs and velocity targets. Their interpretation depends on actuator type. |
| `data.sim.time` | Simulation time in seconds. |
| `data.size.ncon` | Number of detected contact features. |
| `data.con.feature` | Contact features containing geometry identifiers, signed gap, normal, and a surface position on each geometry. |

Read contact results after completing a simulation step. A contact feature count
is not the number of geometry pairs; one pair may produce multiple features. See
[Collision detection](model-construction.md#collision-detection) for how the
features are generated and interpreted.

## Application and control

`AppManager::setControl` installs a control callable:

```cpp
template <typename Callable>
void setControl(Callable&& callable);
```

The callable must accept the following arguments:

```cpp
void control(
  model_t const& m, data_t const& d,
  Ref<VectorXr> u, Ref<VectorXr> udot);
```

The callback writes actuator inputs for the current step. Install it before
`init()`, keep captured objects alive for as long as the callback uses them,
and avoid acquiring the application lock from inside the callback.

```cpp
void init(char const* title = "CRISP🍟", int width = 1200, int height = 900);
bool isOpen() const;
void render();
void shutdown();

AppLock lock();
model_t const& model() const;
model_t& model();
data_t const& data() const;
data_t& data();
```

- `init`, `isOpen`, `render`, and `shutdown` manage the viewer and simulation
  engine.
- `lock` returns an RAII lock for synchronized access to the model and data.
- `model` and `data` provide const and mutable accessors.

When accessing live model/data outside the control callback, hold the lock for
the duration of that access:

```cpp
{
  auto lock = app->lock();
  const auto time = app->data().sim.time;
  // Use or copy the required state while the lock is held.
}
```

## Custom SDF registration

```cpp
using sdf_eval_fn = int (*)(
  Ref<Vector3r const> x_rel, Ref<VectorXr const> param, int requested,
  real_t& phi, Ref<Vector3r> grad, Ref<Matrix3r> hess);
using sdf_aabb_fn = void (*)(Ref<VectorXr const> param, Ref<Vector6r> aabb);

struct sdf_impl_t {
  char const* name = nullptr;
  sdf_eval_fn eval = nullptr;
  sdf_aabb_fn aabb = nullptr;
  int nparam = 0;
};

int register_sdf(sdf_impl_t const& impl);
```

`x_rel` is the local query point and `param` supplies the geometry parameters.
Write the field value to `phi`. Use the `requested` derivative flags to determine
what is requested, and return the flags actually supplied: `sdf_eval_grad` and/or
`sdf_eval_hess`. The optional bounding-box callback writes minimum x/y/z followed
by maximum x/y/z; if it is omitted, CRISP attempts to estimate the bounds
automatically. `register_sdf` returns the identifier passed to
`Geometry::createSDF`.

The [modeling guide](model-construction.md#custom-distance-functions) explains
the role of these callbacks; the examples provide complete implementations.
