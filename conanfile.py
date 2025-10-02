# conanfile.py
from conan import ConanFile
from conan.tools.cmake import cmake_layout, CMake

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
        "slang_version": ["ANY"],
    }
    
    default_options = {
        "build_tests": True,
        "build_examples": True,
        "build_docs": False,
        "float_64bit": False,
        "int_64bit": False,
        "slang_version": "2025.10.4",
    }

    # 生成 CMake 所需文件 & 虚拟环境（给 cmake/ninja/运行时 PATH）
    generators = "CMakeToolchain", "CMakeDeps", "VirtualBuildEnv", "VirtualRunEnv"

    def layout(self):
        # 标准 CMake 布局（单配置会生成 build/<type>，多配置生成 build/）
        cmake_layout(self)

    # 第三方依赖
    def requirements(self):
        self.requires("glfw/3.4")
        self.requires("assimp/5.4.3")
        self.requires("imgui/1.92.2b")
        self.requires("stb/cci.20230920")
        # Vulkan 由 Conan 管理
        self.requires("vulkan-loader/[>=1.3]")

        ver = str(self.options.slang_version)
        self.requires(f"slang/{ver}")  # 你的本地 recipe：conan create . --name=slang --version=<ver>
        
        if self.options.build_tests:
            self.requires("gtest/[>=1.14 <2]")

    # 构建工具
    def build_requirements(self):
        self.tool_requires("cmake/3.27.9")
        self.tool_requires("ninja/1.12.1")

    # 一键构建：conan build .
    def build(self):
        cmake = CMake(self)

        # 把 Conan 选项传入 CMake（与你的顶层 CMakeLists 变量对齐）
        variables = {
            "PBPT_BUILD_TESTS": "ON" if self.options.build_tests else "OFF",
            "PBPT_BUILD_EXAMPLES": "ON" if self.options.build_examples else "OFF",
            "PBPT_BUILD_DOCUMENTATION": "ON" if self.options.build_docs else "OFF",
            "PBPT_FLOAT_64BIT": "ON" if self.options.float_64bit else "OFF",
            "PBPT_INT_64BIT": "ON" if self.options.int_64bit else "OFF"
        }

        # 单配置生成器在配置阶段写入 CMAKE_BUILD_TYPE
        if not cmake.is_multi_configuration:
            variables["CMAKE_BUILD_TYPE"] = str(self.settings.build_type)

        cmake.configure(variables=variables)

        # 多配置生成器在 build 阶段指定构建类型
        if cmake.is_multi_configuration:
            cmake.build(build_type=str(self.settings.build_type))
        else:
            cmake.build()