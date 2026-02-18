# Technology Stack
PBPT (Physically Based Path Tracer) uses a modern, performant technology stack to facilitate advanced rendering research.

# Core Development
- **Language:** C++23 (required for modern features like concepts, ranges, and improved templates).
- **Build System:** CMake 3.27+ with Ninja generator for fast, reproducible builds.
- **Dependency Management:** Conan 2.x, managed within a Python virtual environment using `uv` to ensure consistent and isolated dependencies.

# Performance & Rendering
- **Acceleration:** Intel Embree 4.x for efficient ray-mesh intersection tests.
- **Parallelism:** Intel oneTBB for multi-threaded CPU rendering.
- **Image I/O:** OpenEXR 3.x for high-dynamic-range image output.
- **Math Library:** Custom `pbpt::math` library tailored for rendering tasks.

# Tooling & Verification
- **Scene Loading:** PugiXML for XML-based scene description files.
- **Testing Framework:** GoogleTest for unit and integration testing.
- **Documentation:** MkDocs and Doxygen for creating and serving project documentation.
