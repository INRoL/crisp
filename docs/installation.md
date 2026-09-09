# Installation

## Requirements

- CMake 3.20 or later
- A C++20 compiler
- Windows x86-64, Linux x86-64 or AArch64, or macOS
- An OpenGL-capable desktop for the viewer
- An internet connection during the first CMake configuration

## Build

Clone the repository and build the examples:

```sh
git clone https://github.com/INRoL/crisp.git
cd crisp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

CMake downloads the latest CRISP package for the current platform and Eigen
3.4.0. To use an installed Eigen package, set
`crisp_fetch_eigen=OFF`.

## Run

Run from the repository root so the examples can find `assets/`.

```sh
./build/examples/peg_insertion
./build/examples/gear_assembly
```

With Visual Studio, use the configuration subdirectory:

```powershell
.\build\examples\Release\peg_insertion.exe
.\build\examples\Release\gear_assembly.exe
```

See [Examples](examples.md) for model descriptions and viewer controls.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Missing symbols or API errors | Use matching CRISP headers, binaries, and examples; configure in a fresh build directory. |
| Eigen version error | Use Eigen 3.4.0 or enable `crisp_fetch_eigen`. |
| Missing model or mesh | Run from the repository root with the complete `assets/` directory. |
| Viewer cannot open | Check that a working OpenGL desktop is available. |
| Checksum error | Configure again to retry the download; check the latest release archive and checksum if it repeats. |
