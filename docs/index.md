# PBPT

PBPT is a physically based path tracer implemented in modern C++.

## Documentation

- [Overview](overview.md)
- [Get Started](get-started.md)
- [API Reference](api/index.md)

## Local API docs build

```bash
uv run conan install . \
  -pr:h=profiles/pbpt -pr:b=profiles/pbpt \
  -c tools.build:skip_test=True \
  -c tools.cmake.cmaketoolchain:generator='Unix Makefiles' \
  -s build_type=Release -s compiler.cppstd=23 --build=missing

cmake -S . -B build/Release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake \
  -DPBPT_BUILD_TESTS=OFF \
  -DPBPT_BUILD_EXAMPLES=OFF \
  -DPBPT_BUILD_DOCUMENTATION=ON
cmake --build build/Release --target pbpt_docs
```

## API reference

- [PBPT API (Doxygen)](api/doxygen/html/index.html)
