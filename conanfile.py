# conanfile.py
from conan import ConanFile
from conan.tools.cmake import cmake_layout, CMake, CMakeToolchain
from conan.tools.files import copy
import os

required_conan_version = ">=2.0"

class PbptDeps(ConanFile):
    name = "pbpt-deps"
    version = "0.1"
    settings = "os", "arch", "compiler", "build_type"

    # 项目级可配置项（通过 -o pbpt-deps:<option>=... 覆盖）
    options = {
        "build_tests": [True, False],
        "build_examples": [True, False],
        "build_docs": [True, False],
        "float_64bit": [True, False],
        "int_64bit": [True, False],
        "slang_version": ["ANY"]
    }

    default_options = {
        "build_tests": True,
        "build_examples": True,
        "build_docs": False,
        "float_64bit": False,
        "int_64bit": False,
        "slang_version": "2025.10.4"
    }

    # 生成 CMake 所需文件 & 虚拟环境（给 cmake/ninja/运行时 PATH）
    generators = "CMakeDeps", "VirtualBuildEnv", "VirtualRunEnv"

    def layout(self):
        # ⚠️ 强制使用 Ninja（单配置），这样 CMAKE_EXPORT_COMPILE_COMMANDS 才会生成文件
        cmake_layout(self, generator="Ninja")

    # 第三方依赖
    def requirements(self):
        self.requires("glfw/3.4")
        self.requires("assimp/5.4.3")
        self.requires("imgui/1.92.2b")
        self.requires("stb/cci.20230920")
        self.requires("vulkan-loader/[>=1.3]")

        # 本地 recipe：conan create . --name=slang --version=<ver>
        ver = str(self.options.slang_version)
        self.requires(f"slang/{ver}")  

        if self.options.build_tests:
            self.requires("gtest/[>=1.14 <2]")

    # 构建工具
    def build_requirements(self):
        self.tool_requires("cmake/3.27.9")
        self.tool_requires("ninja/1.12.1")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.generate()

    # 一键构建：conan build .
    def build(self):
        cmake = CMake(self)

        variables = {
            "PBPT_BUILD_TESTS": "ON" if self.options.build_tests else "OFF",
            "PBPT_BUILD_EXAMPLES": "ON" if self.options.build_examples else "OFF",
            "PBPT_BUILD_DOCUMENTATION": "ON" if self.options.build_docs else "OFF",
            "PBPT_FLOAT_64BIT": "ON" if self.options.float_64bit else "OFF",
            "PBPT_INT_64BIT": "ON" if self.options.int_64bit else "OFF",
            "CMAKE_BUILD_TYPE": str(self.settings.build_type),
        }

        cmake.configure(variables=variables)
        cmake.build()