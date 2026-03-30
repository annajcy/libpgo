from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import get


class ArpackNGConan(ConanFile):
    name = "arpack-ng"
    version = "3.9.1"
    license = "BSD-3-Clause"
    url = "https://github.com/opencollab/arpack-ng"
    description = "Collection of Fortran77 subroutines for large scale eigenvalue problems"
    topics = ("eigenvalue", "sparse-matrix", "numerical", "fortran")

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def validate(self):
        # ARPACK-NG requires a Fortran compiler (gfortran).
        # It is primarily useful on Linux with GCC.
        pass

    def layout(self):
        cmake_layout(self, src_folder="src")

    def source(self):
        get(self, **self.conan_data["sources"][self.version], strip_root=True)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
        tc.variables["MPI"] = False
        tc.variables["ICB"] = False
        tc.variables["EIGEN"] = False
        tc.variables["PYTHON3"] = False
        tc.variables["EXAMPLES"] = False
        tc.variables["TESTS"] = False
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "arpackng")
        self.cpp_info.set_property("cmake_target_name", "ARPACK::ARPACK")
        self.cpp_info.libs = ["arpack"]

        if self.settings.os in ("Linux", "FreeBSD"):
            self.cpp_info.system_libs = ["m", "gfortran"]
