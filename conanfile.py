from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import copy
import os

required_conan_version = ">=2.0"


class PbptConan(ConanFile):
    name = "pbpt"
    package_type = "static-library"

    def set_version(self):
        cli_version = getattr(self, "version", None)
        if cli_version:
            return
        self.version = os.getenv("PBPT_VERSION", "0.1.0-dev.local")

    settings = "os", "arch", "compiler", "build_type"
    options = {
        "with_tests": [True, False],
        "with_examples": [True, False],
        "with_docs": [True, False],
        "float_64bit": [True, False],
        "int_64bit": [True, False],
    }
    default_options = {
        "with_tests": True,
        "with_examples": True,
        "with_docs": True,
        "float_64bit": False,
        "int_64bit": False,
        "embree/*:shared": True,
        "hwloc/*:shared": True,
    }

    generators = "CMakeDeps", "VirtualBuildEnv", "VirtualRunEnv"

    def layout(self):
        cmake_layout(self, generator="Ninja")

    def validate(self):
        cppstd = str(self.settings.get_safe("compiler.cppstd") or "")
        if cppstd not in ("23", "gnu23"):
            raise ConanInvalidConfiguration(
                f"pbpt requires compiler.cppstd=23 (or gnu23), got '{cppstd or 'unset'}'"
            )

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
        self.requires("stb/cci.20230920")
        self.requires("pugixml/1.14")
        self.requires(
            "openexr/3.2.4",
            transitive_headers=True,
            transitive_libs=True,
        )
        self.requires(
            "embree/4.4.0",
            transitive_headers=True,
            transitive_libs=True,
        )
        self.requires(
            "onetbb/2021.12.0",
            transitive_headers=True,
            transitive_libs=True,
        )

        if self.options.with_tests:
            self.test_requires("gtest/[>=1.14 <2]")

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
        self.cpp_info.components["rgb_spectrum_lut"].libs = ["pbpt_rgb_spectrum_lut"]

        stb_component = self.cpp_info.components["stb_impl"]
        stb_component.libs = ["pbpt_stb_impl"]
        stb_component.requires = ["stb::stb"]

        core_component = self.cpp_info.components["core"]
        core_component.set_property("cmake_target_name", "pbpt::pbpt")
        core_component.requires = [
            "rgb_spectrum_lut",
            "stb_impl",
            "openexr::openexr",
            "pugixml::pugixml",
            "embree::embree",
            "onetbb::onetbb",
        ]
