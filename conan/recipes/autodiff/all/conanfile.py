from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import get


class AutodiffConan(ConanFile):
    name = "autodiff"
    version = "1.1.2"
    license = "MIT"
    url = "https://github.com/autodiff/autodiff"
    description = "C++17 library for automatic differentiation"
    topics = ("automatic-differentiation", "derivatives", "header-only")

    settings = "os", "arch", "compiler", "build_type"
    no_copy_source = True

    def requirements(self):
        self.requires("eigen/3.4.0")

    def layout(self):
        cmake_layout(self, src_folder="src")

    def source(self):
        get(self, **self.conan_data["sources"][self.version], strip_root=True)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["AUTODIFF_BUILD_TESTS"] = False
        tc.variables["AUTODIFF_BUILD_PYTHON"] = False
        tc.variables["AUTODIFF_BUILD_EXAMPLES"] = False
        tc.variables["AUTODIFF_BUILD_DOCS"] = False
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "autodiff")
        self.cpp_info.set_property("cmake_target_name", "autodiff::autodiff")
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []

    def package_id(self):
        self.info.clear()
