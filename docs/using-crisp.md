# Using CRISP

Include `<crisp/crisp.hpp>` and link against `crisp::crisp`. The CRISP package
headers contain the full declarations.

## Link a program

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_simulation LANGUAGES CXX)
find_package(crisp CONFIG REQUIRED)
add_executable(my_simulation main.cpp)
target_compile_features(my_simulation PRIVATE cxx_std_20)
target_link_libraries(my_simulation PRIVATE crisp::crisp)
```

Configure with `crisp_DIR` pointing to the package's `lib/cmake/crisp` directory
and Eigen 3.4.0 available to CMake.

## Build and step a model

This program simulates a falling sphere without opening a window. Dimensions
are in metres, masses in kilograms, and time in seconds.

```cpp
#include <crisp/crisp.hpp>
#include <cstdio>

int main() {
  auto builder = crisp::make_model_builder();
  builder->opt().dt = 0.005;
  builder->opt().sol.type = crisp::solver_e::canal;

  auto& body = builder->addBody(
    {.name = "sphere",
     .pos = {0.0, 0.0, 0.2},
     .mass = 0.1,
     .inertia = {0.000016, 0.000016, 0.000016, 0, 0, 0}},
    false);
  body.addGeom({.type = crisp::geometry_e::sphere,
                .size = crisp::make_sphere_size(0.02)});

  auto model = builder->build();
  auto data = crisp::make_data(*model);
  crisp::reset(*model, *data);
  for (int i = 0; i < 1000; ++i) {
    crisp::step(*model, *data);
  }
  std::printf("Time: %.3f s; contacts: %d\n",
              data->sim.time, data->size.ncon);
  return data->state.q().allFinite() ? 0 : 1;
}
```

`addBody(..., false)` makes the body free to move. `build()` produces the
model, and `make_data()` allocates its working state. Keep the model alive while
using its data.

## Open the viewer

To display a built model instead of stepping it yourself:

```cpp
// model is the handle returned by builder->build().
auto app = crisp::make_app(std::move(model));
app->init("My simulation", 1280, 800);
while (app->isOpen()) {
  app->render();
}
```

`make_app` takes ownership of the model. The application manages simulation
stepping; `render()` updates the viewer. Set control callbacks before `init()`.

## Control a model

For direct stepping, write actuator inputs through `data.act.u()` and
`data.act.udot()` before calling `step`. A feedback controller that needs refreshed
kinematics can run between `step1` and `step2`; see the
[API reference](api-reference.md#stepping-and-initialization) for their order.

With `AppManager`, install a callback before `init()`:

```cpp
app->setControl([](auto const& model, auto const& data, auto u, auto udot) {
  // Fill u and udot for this model's actuators.
});
```

The lambda shows the callback signature. Position actuators take position and
velocity targets; force actuators take generalized forces. Size and order the
inputs to match the model. See [Actuation](model-construction.md#actuation) for
the input conventions.

See [Examples](examples.md) for time-dependent controllers. Captured objects
must remain alive while the application can call the controller.

## Inspect and visualize

After a direct step, read `data.state.q()`, `data.state.v()`, and
`data.con.feature` to inspect the computed state and contacts. In a running
application, acquire `app->lock()` before accessing its live model or data
outside the control callback.

The viewer can also display contact points, contact vectors, colliders, and
bounding boxes. The `Scene` interface supports applications that manage their
own graphics context.

See [API reference](api-reference.md) for operation signatures and ownership rules.
