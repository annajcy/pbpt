# PBPT

PBPT 是一个使用现代 C++ 实现的基于物理的路径追踪渲染器。

## 文档

- [概览](overview.md)
- [快速上手](get-started.md)
- [API 参考](api/index.md)

## 本地 API 文档构建

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

## API 参考

- [PBPT API（Doxygen）](api/doxygen/html/index.html)
