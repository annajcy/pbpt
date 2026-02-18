# Initial Concept
PBPT (Physically Based Path Tracer) is a modern, extensible path tracing engine built with C++23. It aims to provide a clean, modular architecture for rendering research and serves as the high-quality offline rendering backbone for the RTR2 project.

# Product Vision
PBPT is designed to be a state-of-the-art research renderer that balances performance with high extensibility. By leveraging modern C++23 features and a plugin-oriented architecture, it empowers researchers to rapidly implement and evaluate new rendering algorithms while maintaining the robustness required for production-quality reference images.

# Target Audience
- **Rendering Researchers:** Seeking a modern, performant framework for implementing and testing new algorithms (e.g., advanced light transport).
- **Graphics Engineers:** Using PBPT as a ground-truth reference for real-time rendering development in RTR2.

# Core Goals
- **Feature Parity:** Provide a comprehensive suite of standard rendering components, including diverse BSDFs, light types, and robust integrators.
- **Advanced Light Transport:** Implement sophisticated algorithms like Bidirectional Path Tracing (BDPT) and Metropolis Light Transport (MLT) to handle complex lighting scenarios.
- **RTR2 Integration:** Maintain tight synchronization with the RTR2 project, ensuring shared scene descriptions and data formats for seamless reference rendering.

# Architectural Principles
- **Modern C++23:** Extensive use of templates, concepts, and ranges to ensure type safety, performance, and clear interfaces.
- **Extensible Plugin System:** A modular design that allows for the easy addition of new shapes, materials, lights, and integrators.
- **Low Friction:** Minimal dependencies and a streamlined build process (CMake + Conan + uv) to facilitate easy adoption and integration.
- **Verification & Correctness:** A strong emphasis on automated testing and comparison against known analytical solutions or reference renderers.
