from conan import ConanFile
from conan.tools.files import get, copy, rmdir
from conan.errors import ConanInvalidConfiguration
import os
import glob

class SlangRecipe(ConanFile):
    name = "slang"
    version = "2025.10.4"
    settings = "os", "arch", "compiler", "build_type"
    no_copy_source = True

    def _archive_url(self):
        v = self.version
        sys = str(self.settings.os)
        arch = str(self.settings.arch)

        if sys == "Windows" and arch == "x86_64":
            filename = f"slang-{v}-windows-x86_64.zip"
        elif sys == "Macos":
            if arch in ("armv8", "arm64", "aarch64"):
                filename = f"slang-{v}-macos-aarch64.zip"
            else:
                filename = f"slang-{v}-macos-x86_64.zip"
        elif sys == "Linux" and arch == "x86_64":
            filename = f"slang-{v}-linux-x86_64.zip"
        else:
            raise ConanInvalidConfiguration(f"Unsupported platform: {sys} {arch}")

        return f"https://github.com/shader-slang/slang/releases/download/v{v}/{filename}"

    def package(self):
        # 1) 解压到临时目录（不 strip）
        tmp_src = os.path.join(self.build_folder, "slang_src")
        url = self._archive_url()
        get(self, url, destination=tmp_src, strip_root=False)

        # 2) 处理顶层目录结构差异
        #    - 若只有一个子目录，则进入那个子目录
        #    - 若根下直接是 bin/lib/include，则直接用根
        entries = [os.path.join(tmp_src, e) for e in os.listdir(tmp_src) if not e.startswith(".")]
        root = tmp_src
        if len(entries) == 1 and os.path.isdir(entries[0]):
            root = entries[0]

        # 3) 拷贝 include/lib/bin 到最终包
        inc_dir = os.path.join(root, "include")
        lib_dir = os.path.join(root, "lib")
        bin_dir = os.path.join(root, "bin")

        if os.path.isdir(inc_dir):
            copy(self, pattern="*", src=inc_dir, dst=os.path.join(self.package_folder, "include"))
        # 库文件（不同平台后缀）
        if os.path.isdir(lib_dir):
            copy(self, pattern="*.lib",  src=lib_dir, dst=os.path.join(self.package_folder, "lib"), keep_path=False)
            copy(self, pattern="*.a",    src=lib_dir, dst=os.path.join(self.package_folder, "lib"), keep_path=False)
            copy(self, pattern="*.so*",  src=lib_dir, dst=os.path.join(self.package_folder, "lib"), keep_path=False)
            copy(self, pattern="*.dylib",src=lib_dir, dst=os.path.join(self.package_folder, "lib"), keep_path=False)

        if os.path.isdir(bin_dir):
            copy(self, pattern="*", src=bin_dir, dst=os.path.join(self.package_folder, "bin"), keep_path=False)
        else:
            # 有些发行包把可执行文件放在根或其它目录，兜底把 slangc* 拷到 bin
            for glb in ("**/slangc", "**/slangc.exe", "**/slangc*"):
                for f in glob.glob(os.path.join(root, glb), recursive=True):
                    if os.path.isfile(f):
                        copy(self, pattern=os.path.basename(f), src=os.path.dirname(f),
                             dst=os.path.join(self.package_folder, "bin"), keep_path=False)

        # 可选清理
        rmdir(self, os.path.join(self.package_folder, "share"))

    def package_info(self):
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.libdirs = ["lib"]
        self.cpp_info.bindirs = ["bin"]
        # 主要库名（如发布包里还有其它组件，可追加）
        self.cpp_info.libs = ["slang"]

        # CMake 包信息
        self.cpp_info.set_property("cmake_file_name", "slang")
        self.cpp_info.set_property("cmake_target_name", "slang::slang")

        # 运行环境，方便直接调用 slangc
        self.runenv_info.append_path("PATH", os.path.join(self.package_folder, "bin"))
