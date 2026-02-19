# Copilot Instructions for PBPT

## Build, test, and docs commands

PBPT uses `uv` + Conan + CMake presets; prefer these commands.

```bash
# 1) Python/Conan tooling
uv sync
uv run conan profile detect --force

# 2) (Optional, when conan_recipe/ exists) build local Conan recipes
(cd conan_recipe && uv run python build_conan_recipes.py -d . -b Debug -v)

# 3) Install deps/toolchain for Debug
uv run conan install . -pr:h=profiles/pbpt -pr:b=profiles/pbpt -s build_type=Debug --build=missing

# 4) Configure + build
cmake --preset conan-debug
cmake --build --preset conan-debug
```

Run all tests:

```bash
source build/Debug/generators/conanrun.sh
ctest --test-dir build/Debug -C Debug --output-on-failure
```

Run a single test (example):

```bash
./build/Debug/test/loader/test_scene_loader_matrix \
  --gtest_filter=SceneLoaderMatrixTest.LoadsShapeMatrixTransform
```

Docs:

```bash
uv run mkdocs build
uv run mkdocs serve
```

Linting:
- No dedicated lint target is defined in CMake/pyproject.
- Formatting rules are defined in `.clang-format`.

## High-level architecture

- `src/CMakeLists.txt` builds `pbpt` as an interface target and links core dependencies (`OpenEXR`, `pugixml`, `embree`, `oneTBB`), plus a generated static LUT library (`pbpt_rgb_spectrum_lut`) from `script/generate_rgb_spectrum_lut.py`.
- Scene loading is centered in `pbpt::loader::load_scene` (`src/pbpt/loader/scene_loader.hpp`): XML is parsed into material/light/spectrum/mesh libraries, then flattened into triangle `Primitive`s and wrapped in `EmbreeAggregate`.
- Runtime scene data is stored in `pbpt::scene::Scene<T>` (`src/pbpt/scene/scene.hpp`) using plugin-like `Any*` variants (camera/film/filter/aggregate) plus `RenderResources` named libraries.
- Rendering flow is `examples/render_scene.cpp` -> `PathIntegrator::render(...)` (`src/pbpt/integrator/integrator.hpp`) -> recursive path tracing implementation in `path_integrator.hpp`.

## Key conventions specific to this repo

- Keep code under `pbpt::<module>` namespaces, with templates typically parameterized as `<T>` (scalar precision) and sometimes `<T, N>` (sampled-spectrum lanes).
- Loader/plugin wiring is name-driven: BSDFs, spectra, meshes, and lights are registered in named libraries and then referenced by IDs/strings from scene XML.
- Mesh-emissive light mapping keys must remain `"<mesh_name>_<triangle_index>"`; loader creation and aggregate lookup both depend on this exact format.
- Render-space handling goes through `camera::RenderTransform`; object transforms should be converted with `object_to_render_from_object_to_world(...)` before mesh construction.
- Roughness parsing in the scene loader accepts multiple aliases (`alpha`, `alpha_x/y`, `alphaU/V`, `roughness*`) and converts roughness to microfacet alpha via `sqrt`.
- Tests are module-local under `test/<module>/`, built as `test_*` executables, and registered with CTest via `gtest_discover_tests(...)`.
