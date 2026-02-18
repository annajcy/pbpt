# Get Started

This guide helps you build PBPT and run your first example.

## Prerequisites

- C++ compiler with C++23 support
- CMake (3.25+ recommended)
- Conan 2.x
- Python and `uv` (for docs tooling)

## 1) Install dependencies (Conan)

```bash
uv run conan install . \
  -pr:h=profiles/pbpt -pr:b=profiles/pbpt \
  -c tools.build:skip_test=True \
  -c tools.cmake.cmaketoolchain:generator='Unix Makefiles' \
  -s build_type=Release -s compiler.cppstd=23 --build=missing
```

## 2) Configure and build

```bash
cmake -S . -B build/Release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake

cmake --build build/Release -j
```

## 3) Run an example

After building, run one of the example executables from `build/Release/examples/` (name depends on your generator/platform), then check rendered outputs under `output/`.

## 4) Build and preview docs

```bash
uv run mkdocs build
uv run mkdocs serve
```

Open `http://127.0.0.1:8000` to preview the documentation site.

## 5) API docs

To build Doxygen API docs:

```bash
cmake -S . -B build/Release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake \
  -DPBPT_BUILD_TESTS=OFF \
  -DPBPT_BUILD_EXAMPLES=OFF \
  -DPBPT_BUILD_DOCUMENTATION=ON

cmake --build build/Release --target pbpt_docs
```

Then open the generated API entry under `docs/api/doxygen/html/index.html`.
