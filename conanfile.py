from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import copy

required_conan_version = ">=2.0"


class PbptConan(ConanFile):
    name = "pbpt"
    version = "0.1.0"
    package_type = "library"

    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_tests": [True, False],
        "with_examples": [True, False],
        "with_docs": [True, False],
        "float_64bit": [True, False],
        "int_64bit": [True, False],
        "slang_version": ["ANY"],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "with_tests": False,
        "with_examples": False,
        "with_docs": False,
        "float_64bit": False,
        "int_64bit": False,
        "slang_version": "2025.10.4",
        "embree/*:shared": True,
    }

    generators = "CMakeDeps", "VirtualBuildEnv", "VirtualRunEnv"

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self, generator="Ninja")

    def export_sources(self):
        copy(
            self,
            "*",
            src=self.recipe_folder,
            dst=self.export_sources_folder,
            excludes=[
                ".git/*",
                ".github/*",
                ".vscode/*",
                "build/*",
                "**/build/*",
                "output/*",
                "**/output/*",
                "docs/*",
                "pbpt_docs/*",
                "**/__pycache__/*",
                "*.pyc",
                "CMakeUserPresets.json",
                "compile_commands.json",
            ],
        )

    def requirements(self):
        self.requires("glfw/3.4")
        self.requires("assimp/5.4.3")
        self.requires("imgui/1.92.2b")
        self.requires("stb/cci.20230920")
        self.requires("vulkan-loader/[>=1.3]")
        self.requires("openexr/3.2.4")
        self.requires("pugixml/1.14")
        self.requires("embree/4.4.0")
        self.requires("onetbb/2021.12.0")

        ver = str(self.options.slang_version)
        self.requires(f"slang/{ver}")

        if self.options.with_tests:
            self.requires("gtest/[>=1.14 <2]")

    def build_requirements(self):
        self.tool_requires("cmake/3.27.9")
        self.tool_requires("ninja/1.12.1")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.cache_variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = True
        tc.generate()

    def build(self):
        cmake = CMake(self)
        variables = {
            "PBPT_BUILD_TESTS": "ON" if self.options.with_tests else "OFF",
            "PBPT_BUILD_EXAMPLES": "ON" if self.options.with_examples else "OFF",
            "PBPT_BUILD_DOCUMENTATION": "ON" if self.options.with_docs else "OFF",
            "PBPT_FLOAT_64BIT": "ON" if self.options.float_64bit else "OFF",
            "PBPT_INT_64BIT": "ON" if self.options.int_64bit else "OFF",
            "CMAKE_BUILD_TYPE": str(self.settings.build_type),
        }
        cmake.configure(variables=variables)
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "pbpt")
        self.cpp_info.set_property("cmake_target_name", "pbpt::pbpt")
        self.cpp_info.libs = ["pbpt_rgb_spectrum_lut"]
