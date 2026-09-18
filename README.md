# CRISP

**A physics engine for contact-rich robotic assembly.**

CRISP uses expressive collision geometries and robust contact solvers to simulate
multibody dynamics with frictional contact.

This repository provides two assembly examples and CMake integration for the
prebuilt CRISP package.

[Project page](https://inrol.github.io/crisp/) ·
[Documentation](https://inrol.github.io/crisp/docs/overview/)

![Gear assembly simulation](site/assets/gear_assembly.gif)

## Getting started

You need CMake 3.20 or later, a C++20 compiler, and an OpenGL-capable desktop.

CMake downloads the latest CRISP package and Eigen during configuration.

```sh
git clone https://github.com/INRoL/crisp.git
cd crisp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Run the examples from the repository root so that their relative paths to
`assets/` resolve correctly:

```sh
./build/examples/peg_insertion
./build/examples/gear_assembly
```

See [Installation](docs/installation.md) for other configurations and
troubleshooting, and [Examples](docs/examples.md) for model and viewer
details.

## Example scenarios

| Scenario | Description | Solver |
| --- | --- | --- |
| `peg_insertion` | Grasp, lift, and insert a round peg into a closely fitting hole. | CANAL |
| `gear_assembly` | Place a gear on its shaft, then turn it between neighboring gears. | SubADMM |

Both examples use a Franka arm with a Robotiq 2F-85 gripper and prescribed joint
targets.

## Publications

- S. Lee, S. Park, J. Yun, S. An, and D. J. Lee, "[CRISP: Contact-Rich Robotic Simulation Platform with Extensive Geometries and Contact Solvers](https://inrol.github.io/crisp/papers/crisp-preprint.pdf)," arXiv preprint, 2026.
- J. Lee, M. Lee, S. Park, J. Yun, and D. J. Lee, "[Variations of Augmented Lagrangian for Robotic Multi-Contact Simulation](https://doi.org/10.1109/TRO.2025.3577410)," IEEE Transactions on Robotics, 2025.
- J. Lee, M. Lee, and D. J. Lee, "[Modular and Parallelizable Multibody Physics Simulation via Subsystem-Based ADMM](https://doi.org/10.1109/ICRA48891.2023.10161052)," IEEE International Conference on Robotics and Automation, 2023.
- H. Ji, H. Kim, J. Lee, S. Lee, S. An, J. Heo, Y. Lee, Y. Lee, and D. J. Lee, "[GPU-Accelerated Subsystem-Based ADMM for Large-Scale Interactive Simulation](https://doi.org/10.1109/ICRA55743.2025.11128665)," IEEE International Conference on Robotics and Automation, 2025.
- J. Lee, M. Lee, and D. J. Lee, "[Uncertain Pose Estimation during Contact Tasks Using Differentiable Contact Features](https://doi.org/10.15607/RSS.2023.XIX.043)," Robotics: Science and Systems, 2023.
- S. An, S. Lee, J. Lee, S. Park, and D. J. Lee, "[Collision Detection between Smooth Convex Bodies via Riemannian Optimization Framework](https://doi.org/10.1109/IROS58592.2024.10802286)," IEEE/RSJ International Conference on Intelligent Robots and Systems, 2024.

## Citation

If you use CRISP in your research, please cite this repository and the
[relevant method papers](docs/publications.md#related-publications).

```bibtex
@misc{crisp2026,
  title = {{CRISP}: Contact-Rich Robotic Simulation Platform with Extensive Geometries and Contact Solvers},
  author = {{Interactive \& Networked Robotics Laboratory (INRoL)}},
  year = {2026},
  url = {https://github.com/INRoL/crisp}
}
```

## Contact

For questions or feedback, please contact
[hopelee@snu.ac.kr](mailto:hopelee@snu.ac.kr).

## License

CRISP is available for academic and non-commercial research under the
[license terms](LICENSE). Third-party assets retain their own
[licenses](docs/licenses.md).
