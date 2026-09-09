# Overview

CRISP is a physics engine for contact-rich robotic assembly. It uses expressive
collision geometries and robust contact solvers to simulate multibody dynamics
with frictional contact. We provide a prebuilt C++ library, headers, and two
examples with the assets required to run them.

## Assembly scenarios

Both models use a Franka arm with a Robotiq 2F-85 gripper.

| Model | Sequence | Contact solver |
| --- | --- | --- |
| Peg insertion | Grasp and lift a round peg, then insert it into a hole. | CANAL |
| Gear assembly | Place a gear on its shaft, release and grasp it again, then turn it in both directions. | SubADMM |

## Get started

Use [Installation](installation.md) to build and run the examples. See
[Examples](examples.md) for their sequences, collision representations, solver
settings, and viewer controls.

## Guides

- [Model construction](model-construction.md): bodies, joints, collision geometry, and actuators
- [Contact solvers](contact-solvers.md): CANAL, SubADMM, and numerical settings
- [Using CRISP](using-crisp.md): C++ model construction, stepping, and control
- [API reference](api-reference.md): operations, ownership, state access, and callbacks

See [Publications](publications.md) for the method papers and evaluations. CRISP
is available for academic and non-commercial research under the
[CRISP license](../LICENSE); third-party assets retain
[their own terms](licenses.md).
