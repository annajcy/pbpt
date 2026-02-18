# Overview

PBPT (Physically Based Path Tracer) is a modern C++ renderer focused on physically-based light transport simulation, modular architecture, and reproducible offline rendering workflows.

## Highlights

- Physically based path tracing pipeline
- Modular scene, material, and spectrum subsystems
- CMake + Conan based build workflow
- Example programs for quick experiments and comparisons
- Doxygen API documentation generation

## Project Structure

- `src/`: core implementation
- `examples/`: runnable sample programs
- `asset/`: models, scenes, and spectral data
- `test/`: test suite
- `docs/`: MkDocs and API documentation entry

## Typical Workflow

1. Configure dependencies and toolchain with Conan.
2. Generate build files with CMake.
3. Build targets (library, examples, docs).
4. Run examples to render images.
5. Inspect and extend APIs via generated documentation.

## Next Step

Continue with [Get Started](get-started.md) for a step-by-step setup and first run.
