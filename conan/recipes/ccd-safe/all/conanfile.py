import os

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import save, load


class CCDSafeConan(ConanFile):
    name = "ccd-safe"
    version = "1.0.0"
    license = "Proprietary"
    description = "Safe continuous collision detection"
    topics = ("collision-detection", "physics")

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

    exports_sources = "src/*", "CMakeLists.txt.in"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self, src_folder="src")

    def source(self):
        cmake_template = load(self, os.path.join(self.source_folder, "..", "CMakeLists.txt.in"))
        save(self, os.path.join(self.source_folder, "CMakeLists.txt"), cmake_template)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "CCD_SafeCCD")
        self.cpp_info.set_property("cmake_target_name", "CCD::SafeCCD")
        self.cpp_info.libs = ["CCD_SafeCCD"]
        self.cpp_info.includedirs = ["include/SafeCCD"]
