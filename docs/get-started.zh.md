# 快速上手

本指南帮助你构建 PBPT 并运行第一个示例。

## 前置依赖

- 支持 C++23 的 C++ 编译器
- CMake（推荐 3.25+）
- Conan 2.x
- Python 及 `uv`（用于文档工具链）

## 1）安装依赖（Conan）

```bash
uv run conan install . \
  -pr:h=profiles/pbpt -pr:b=profiles/pbpt \
  -c tools.build:skip_test=True \
  -c tools.cmake.cmaketoolchain:generator='Unix Makefiles' \
  -s build_type=Release -s compiler.cppstd=23 --build=missing
```

## 2）配置并构建

```bash
cmake -S . -B build/Release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake

cmake --build build/Release -j
```

## 3）运行示例

构建完成后，从 `build/Release/examples/` 中运行相应的示例可执行文件（名称因生成器/平台而异），渲染输出位于 `output/` 目录下。

## 4）构建并预览文档

```bash
uv run mkdocs build
uv run mkdocs serve
```

在浏览器中打开 `http://127.0.0.1:8000` 预览文档站点。

## 5）API 文档

构建 Doxygen API 文档：

```bash
cmake -S . -B build/Release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake \
  -DPBPT_BUILD_TESTS=OFF \
  -DPBPT_BUILD_EXAMPLES=OFF \
  -DPBPT_BUILD_DOCUMENTATION=ON

cmake --build build/Release --target pbpt_docs
```

随后在 `docs/api/doxygen/html/index.html` 查看生成的 API 入口。
