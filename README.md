# Physically Based Path Tracer (PBPT)

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
conan build . -s build_type=Debug -s compiler.cppstd=17 -c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=True --build=missing
```
