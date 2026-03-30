from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import get, replace_in_file


class GmshConan(ConanFile):
    name = "gmsh"
    version = "4.13.1"
    license = "GPL-2.0-or-later"
    url = "https://gmsh.info"
    description = "A three-dimensional finite element mesh generator"
    topics = ("mesh-generation", "finite-element", "CAD")

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": True, "fPIC": True}

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self, src_folder="src")

    def source(self):
        get(self, **self.conan_data["sources"][self.version], strip_root=True)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["ENABLE_BUILD_LIB"] = True
        tc.variables["ENABLE_BUILD_SHARED"] = bool(self.options.shared)
        tc.variables["ENABLE_BUILD_DYNAMIC"] = bool(self.options.shared)
        tc.variables["ENABLE_FLTK"] = False
        tc.variables["ENABLE_OPENGL"] = False
        tc.variables["ENABLE_OCC"] = False
        tc.variables["ENABLE_CGNS"] = False
        tc.variables["ENABLE_MED"] = False
        tc.variables["ENABLE_PRIVATE_API"] = True
        tc.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
        tc.generate()

    def _patch_sources(self):
        # Gmsh 4.13.1 uses cmake_minimum_required(VERSION 2.8) which
        # CMake ≥ 3.30 rejects.  Bump it so the configure step succeeds.
        replace_in_file(
            self,
            f"{self.source_folder}/CMakeLists.txt",
            "cmake_minimum_required(VERSION 3.3 FATAL_ERROR)",
            "cmake_minimum_required(VERSION 3.15 FATAL_ERROR)",
        )

    def build(self):
        self._patch_sources()
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "gmsh")
        self.cpp_info.set_property("cmake_target_name", "Gmsh::Gmsh")
        self.cpp_info.libs = ["gmsh"]

        if self.settings.os in ("Linux", "FreeBSD"):
            self.cpp_info.system_libs = ["m", "dl", "pthread"]
