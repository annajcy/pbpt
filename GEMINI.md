# GEMINI.md - Project Context for PBPT

## Project Overview
PBPT (Physically Based Path Tracer) is a modern, extensible path tracing engine built with **C++23**. It is inspired by pbrt but focuses on modern C++ features and provides a clean, modular architecture for research and rendering. It serves as the offline rendering backbone for the RTR2 project.

### Key Technologies
- **Language:** C++23 (required).
- **Acceleration:** Intel Embree 4.x for ray-mesh intersection.
- **Parallelism:** Intel oneTBB for multi-threaded rendering.
- **Image I/O:** OpenEXR 3.x for high-dynamic-range output.
- **Scene Loading:** PugiXML for XML-based scene descriptions.
- **Dependency Management:** Conan 2.0+ (managed via `uv` for Python environment).
- **Build System:** CMake 3.27+ with Ninja generator.
- **Testing:** GoogleTest.

## Architecture
The project is modularized into several key components in `src/pbpt`:
1.  **Radiometry:** Spectral math, color spaces (XYZ, sRGB, LAB), and spectrum distributions (BlackBody, RGB, etc.).
2.  **Math:** Low-level linear algebra (vectors, matrices, transforms), random number generation, and sampling utilities.
3.  **Geometry:** Ray-primitive interactions, bounding boxes, and differential rays.
4.  **Integrator:** Various rendering algorithms (Path tracing, Volpath, etc.).
5.  **Camera:** Camera models (Perspective, Thin Lens, etc.) and film management.
6.  **Material/BSDF:** Physically based materials and scattering functions.
7.  **Shape:** Geometric primitives (Sphere, Triangle, etc.).
8.  **Light:** Area lights and point lights.
9.  **Aggregate:** Acceleration structures (wrappers around Embree).
10. **Loader:** Scene loading logic for XML-based formats.

## Building and Running

### Prerequisites
- Python and `uv` installed.
- C++23-compliant compiler (Clang 16+, GCC 13+, MSVC 2022+).
- CMake 3.27+.

### Build Commands
1.  **Setup Environment:**
    ```bash
    uv sync
    ```
2.  **Install Dependencies & Build:**
    The project uses Conan for dependency management. You can use the provided profiles:
    ```bash
    uv run conan install . -pr=profiles/pbpt -s build_type=Debug --build=missing
    ```
3.  **Configure and Build:**
    ```bash
    cmake --preset conan-debug
    cmake --build --preset conan-debug
    ```

### Running Examples
- **General Render:**
  ```bash
  ./build/Debug/examples/render_scene asset/scene/cbox/cbox.xml 64 output.exr
  ```
- **Other Examples:** Check the `examples/` directory for specialized test cases like `cbox_microfacet`, `white_furnace_compare`, etc.

## Development Conventions

### Coding Style
- **Namespaces:** All code resides under the `pbpt` namespace, further subdivided (e.g., `pbpt::math`, `pbpt::radiometry`).
- **Modern C++:** Use C++23 features extensively (concepts, std::format, etc.).
- **Precision:** Supports both 32-bit and 64-bit precision via CMake options (`PBPT_FLOAT_64BIT`).
- **Templates:** Components are often templated on the precision type (`Float`).

### Testing
- Tests are located in the `test/` directory and use GoogleTest.
- Run tests using `ctest` from the build directory:
  ```bash
  cd build/Debug && ctest
  ```

### Key Files
- `conanfile.py`: Main dependency and build configuration.
- `CMakeLists.txt`: Project build configuration.
- `src/pbpt/pbpt.h`: Main header including common modules.
- `src/pbpt/integrator/plugin/integrator/path_integrator.hpp`: Core path tracing logic.
- `src/pbpt/loader/scene_loader.hpp`: Entry point for scene loading.
- `examples/render_scene.cpp`: Standard CLI entry point for rendering scenes.
