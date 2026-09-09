# Examples

Both examples use a Franka arm with a Robotiq 2F-85 gripper and prescribed joint
targets. [Build them](installation.md), then run them from the repository root.

## Peg insertion

```sh
./build/examples/peg_insertion
```

The gripper grasps and lifts a round peg from a tray, then inserts it into a
closely fitting hole. Contact occurs between the fingers and peg during transport
and between the peg and fixture during insertion.

The peg uses a custom rounded signed distance field (SDF). The tray and hole use
separate visual and collision meshes. The source defines the SDF, robot
actuators, and time-based controller.

See [peg_insertion.cpp](../examples/peg_insertion.cpp) for the complete
model and controller.

## Gear assembly

```sh
./build/examples/gear_assembly
```

The gripper places the middle gear on its shaft, releases and grasps it again,
then turns it in both directions. The bore contacts the shaft while the teeth
contact the adjacent gears, constraining the moving gear at several locations.

All three gears use a parameterized gear-with-hub SDF; the base and shafts use
mesh collision geometry. The SDF represents the teeth, bore, and hub in one
object.

See [gear_assembly.cpp](../examples/gear_assembly.cpp) for the complete model and
controller.

## Simulation settings

| Setting | Peg insertion | Gear assembly |
| --- | --- | --- |
| Solver | CANAL | SubADMM |
| Time step | 5 ms | 5 ms |
| Maximum solver iterations | 20 | 200 |
| Contact ERP | 0.1 | 0.1 |

The controllers follow prescribed joint targets; they do not plan the assembly
motion.
