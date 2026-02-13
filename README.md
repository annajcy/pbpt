# PBPT: A Physically Based Path Tracer 

**WIP**

Yet another pbrt, but with a focus on modern C++ and extensibility.

# How to build

### Install UV

```bash
conda install conda-forge::uv
conda install conda-forge::conan
```
```bash
uv sync 
```

if using macos/linux
```bash
source .venv/bin/activate
```

if using windows
```powershell
.venv\Scripts\activate
```

### Conan Build

```bash
conan profile detect --force
```

```bash
cd conan_recipe
python build_conan_recipes.py -d . -v
cd ..
```

```bash
conan create . --name=pbpt --version=0.1.0-dev.<sha> -s build_type=Debug -s compiler.cppstd=23 -c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=True --build=missing
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

### Version alignment with rtr

When consumed by `rtr`, keep `pbpt` and `rtr` on the same dev version family:

```bash
SHA=$(git rev-parse --short HEAD)
PBPT_VER="0.1.0-dev.${SHA}"
RTR_VER="0.1.0-dev.${SHA}"

conan create external/pbpt --version ${PBPT_VER} -s build_type=Debug -s compiler.cppstd=23 --build=missing
conan create . --name=rtr --version ${RTR_VER} -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 -o "&:pbpt_version=${PBPT_VER}" --build=missing
```

If `rtr` resolves an older `pbpt` from cache (for example `pbpt/0.1.0`), dependency variants can conflict (such as `imgui` vs `imgui-docking`).
