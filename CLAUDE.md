# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PBPT is a header-only, physically-based path tracer in C++23. It is used as a submodule in the RTR2 engine (`external/pbpt`) and can also be built standalone.

## Build Commands

**Setup (first time):**
```bash
uv sync
uv run conan profile detect --force
cd conan_recipe && uv run python build_conan_recipes.py -d . -b Debug -v && cd ..
uv run conan install . -pr=profiles/pbpt -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset conan-debug
cmake --build --preset conan-debug
```

Always use `uv run` for Conan/Python commands (never activate the venv manually).

**Build a specific target:**
```bash
cmake --build build/Debug --target cbox
```

**Run all tests:**
```bash
ctest --test-dir build/Debug -C Debug
```

**Run a single test binary:**
```bash
./build/Debug/test/math/test_vector
```

**CMake options** (pass via `-D` during configure):
- `PBPT_FLOAT_64BIT=ON` — use `double` instead of `float`
- `PBPT_INT_64BIT=ON` — use `int64_t` instead of `int32_t`
- `PBPT_BUILD_TESTS=OFF` — skip test targets
- `PBPT_BUILD_EXAMPLES=OFF` — skip example targets

## Architecture

### Module Layout (`src/pbpt/`)

Each module has an umbrella header (`math.h`, `geometry.h`, etc.) and a `plugin/` subdirectory for concrete implementations.

| Module | Purpose |
|--------|---------|
| `math/` | Core math types: `Vector<T,N>`, `Point<T,N>`, `Normal<T,N>`, `Matrix<T,M,N>`, `Quaternion<T>`, `Homo<T,N>`, `OctahedralVector<T>` |
| `geometry/` | `Ray<T,N>`, `RayDiff<T,N>`, `Bounds<T,N>`, `Transform<T>`, `SurfaceInteraction<T>`, `Frame<T>` |
| `radiometry/` | Spectral rendering: `SampledWavelength<T,N>`, `SampledSpectrum<T,N>`, `ColorSpace`, `SpectrumDistribution` |
| `camera/` | `Camera`, `Film`, `PixelFilter`, `PixelSensor`; plugins for projective/spherical cameras, HDR film, box/tent/gaussian filters |
| `shape/` | CRTP `Shape<Derived,T>` base; plugins for `Sphere`, `Triangle` |
| `aggregate/` | Acceleration structures: `LinearAggregate` (naive), `EmbreeAggregate` (Intel Embree) |
| `integrator/` | `PathIntegrator` (path tracing) via CRTP `Integrator<Derived,T,N,Sampler>` |
| `sampler/` | 1D/2D/3D sampling utilities, discrete sampling |
| `material/` | CRTP `Material<Derived,T>`, `BSDF<T,N>`, `BxDF<T>`; plugins for Lambertian, Dielectric, Conductor (rough/specular variants) |
| `light/` | `AreaLight` via CRTP `Light<Derived,T>` |
| `texture/` | `BitmapTexture`, `Mipmap`, `Image` |
| `scene/` | `Scene`, `CboxScene` |
| `loader/` | Mitsuba-style XML scene parsing (`SceneLoader`, `BasicParser`) |
| `lds/` | Low-discrepancy samplers |
| `utils/` | Image I/O, EXR writer, OBJ loader, progress bar |

### Key Design Patterns

- **Header-only:** All implementation lives in `.hpp` files. `pbpt.h` is an auto-generated umbrella header (do not edit manually—run `script/generate_pbpt_header.py`).
- **CRTP for shapes/materials/lights/integrators:** No virtual dispatch. Each plugin type provides `*_impl()` methods.
- **`std::variant` dispatch:** Used for heterogeneous scene collections (multiple shape/material types in one scene).
- **Configurable precision:** `Float` and `Int` are typedefs controlled by `PBPT_FLOAT_64BIT` / `PBPT_INT_64BIT` preprocessor flags. All math code uses these aliases.

## Coding Conventions

### Naming

| Element | Convention | Example |
|---------|-----------|---------|
| Classes/Structs | PascalCase | `SurfaceInteraction`, `PathIntegrator` |
| Functions/Methods | snake_case | `compute_bsdf()`, `sample_direction()` |
| Member variables | `m_` + snake_case | `m_origin`, `m_film` |
| Namespaces | snake_case | `pbpt::geometry`, `pbpt::material::plugin` |
| Files | snake_case | `sampled_spectrum.hpp`, `scene_loader.hpp` |
| Type aliases | PascalCase | `Vec3`, `Pt3`, `Mat4` |

### Style

- `#pragma once` (not `#ifndef` guards)
- 4-space indentation, K&R brace style, 120-character column limit
- Include order: standard library → third-party → project headers
- Brace-initialize member variables in-class: `int m_width{0};`
- Prefer `constexpr` functions over macros

## Tests

- **Framework:** GoogleTest 1.14+
- **File naming:** `test_{feature}.cpp` (e.g., `test_vector.cpp`)
- **Test namespace:** `pbpt::{module}::test`
- **Test naming:** `TEST(PascalCaseClass, PascalCaseDescription)`
- **Assertions:** `EXPECT_*` preferred (non-fatal); `ASSERT_*` only when continuation is impossible
- **No mocking framework** — use hand-written stubs/probes

### Adding a test

1. Create `test/{module}/test_{feature}.cpp` in `namespace pbpt::{module}::test`
2. Add to `test/{module}/CMakeLists.txt` with `gtest_discover_tests()`
3. Include the standard main boilerplate:
   ```cpp
   int main(int argc, char** argv) {
       ::testing::InitGoogleTest(&argc, argv);
       return RUN_ALL_TESTS();
   }
   ```

## Adding New Plugins

**New shape:** `src/pbpt/shape/plugin/shape/{name}.hpp`, inherit `Shape<Derived, T>`, implement `intersect_impl()`, `bounds_impl()`, `sample_impl()`.

**New material:** `src/pbpt/material/plugin/material/{name}.hpp`, inherit `Material<Derived, T>`, implement `compute_bsdf_impl()`.

**New aggregate:** `src/pbpt/aggregate/plugin/aggregate/{name}.hpp`, inherit `Aggregate<Derived, T>`.

After adding a plugin, update the corresponding `plugin.hpp` and `{type}_type.hpp` files in the same plugin directory to register the variant.
