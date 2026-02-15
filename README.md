# PBPT: A Physically Based Path Tracer 

**WIP**

Yet another pbrt, but with a focus on modern C++ and extensibility.

# How to build

### Install UV

```bash
conda install conda-forge::uv
```
```bash
uv sync 
```

`conan` is installed into the project virtual environment by `uv sync`.

Use `uv run` to execute Python/Conan commands; no manual environment activation is needed.

### Conan Build

`profiles/pbpt` keeps PBPT on C++23 and pins `embree/*:compiler.cppstd=20` for compatibility.

```bash
uv run conan profile detect --force
```

```bash
cd conan_recipe
uv run python build_conan_recipes.py -d . -b Debug -v
cd ..
```

```bash
uv run conan create . --name=pbpt --version=0.1.0-dev.<sha> -pr:h=profiles/pbpt -pr:b=profiles/pbpt -s build_type=Debug --build=missing
```

### Downstream consumption

```python
# conanfile.py
def requirements(self):
    self.requires("pbpt/0.1.0-dev.<sha>")
```

```cmake
find_package(pbpt CONFIG REQUIRED)
target_link_libraries(your_app PRIVATE pbpt::pbpt)
```

### Usage with rtr

`rtr` now vendors `external/pbpt` directly.

```bash
SHA=$(git rev-parse --short HEAD)
RTR_VER="0.1.0-dev.${SHA}"

uv run conan create . --name=rtr --version ${RTR_VER} -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
```

If cache/state is stale after option/profile changes, clear `rtr` cache entries and recreate the package.
