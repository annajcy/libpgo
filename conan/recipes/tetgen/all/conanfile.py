import os

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, save, load


class TetgenConan(ConanFile):
    name = "tetgen"
    version = "1.6.0"
    license = "AGPL-3.0-or-later"
    url = "http://www.tetgen.org"
    description = "A Quality Tetrahedral Mesh Generator and 3D Delaunay Triangulator"
    topics = ("mesh-generation", "tetrahedral", "delaunay")

    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "with_cgal_predicates": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "with_cgal_predicates": False,
    }

    exports_sources = "src/*", "CMakeLists.txt.in"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def requirements(self):
        if self.options.with_cgal_predicates:
            self.requires("cgal/6.0.1")

    def layout(self):
        cmake_layout(self, src_folder="src")

    def source(self):
        # Install the CMakeLists.txt from the template
        cmake_template = load(self, os.path.join(self.source_folder, "..", "CMakeLists.txt.in"))
        save(self, os.path.join(self.source_folder, "CMakeLists.txt"), cmake_template)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["TETGEN_USE_CGAL_PREDICATES"] = bool(
            self.options.with_cgal_predicates
        )
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "tetgen")
        self.cpp_info.set_property("cmake_target_name", "tetgen::tetgen")
        self.cpp_info.libs = ["tetgen"]
        self.cpp_info.defines = ["TETLIBRARY"]
