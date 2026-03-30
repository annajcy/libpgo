from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import get, apply_conandata_patches, export_conandata_patches, replace_in_file


class GeogramConan(ConanFile):
    name = "geogram"
    version = "1.9.0"
    license = "BSD-3-Clause"
    url = "https://github.com/BrunoLevy/geogram"
    description = "Programming library of geometric algorithms"
    topics = ("geometry", "mesh", "voronoi", "delaunay")

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

    def export_sources(self):
        export_conandata_patches(self)

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
        # Geogram 1.9.0's bundled PoissonRecon is broken with modern compilers.
        # Force C++14 (matching libpgo's original build) to minimise breakage.
        tc.variables["CMAKE_CXX_STANDARD"] = 14
        tc.variables["GEOGRAM_SUB_BUILD"] = True
        tc.variables["GEOGRAM_WITH_HLBFGS"] = True
        tc.variables["GEOGRAM_WITH_GRAPHICS"] = False
        tc.variables["GEOGRAM_LIB_ONLY"] = True
        tc.variables["GEOGRAM_WITH_LUA"] = False
        tc.variables["GEOGRAM_WITH_FPG"] = False
        tc.variables["GEOGRAM_WITH_TETGEN"] = False
        tc.variables["GEOGRAM_WITH_TRIANGLE"] = False
        tc.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
        tc.generate()

    def _patch_sources(self):
        apply_conandata_patches(self)
        # Geogram 1.9.0's bundled PoissonRecon has code errors with modern
        # compilers (missing members m_N, value).  PoissonRecon is not used by
        # libpgo, so strip its source files from the third_party build list.
        tp_cmake = f"{self.source_folder}/src/lib/geogram/third_party/CMakeLists.txt"
        replace_in_file(
            self, tp_cmake,
            'aux_source_directories(SOURCES "Source Files\\\\PoissonRecon" PoissonRecon)',
            '# PoissonRecon removed – broken with modern compilers\n'
            '# aux_source_directories(SOURCES "Source Files\\\\PoissonRecon" PoissonRecon)',
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
        self.cpp_info.set_property("cmake_file_name", "geogram")
        self.cpp_info.set_property("cmake_target_name", "geogram::geogram")
        self.cpp_info.includedirs = ["include", "include/geogram1"]
        self.cpp_info.libs = ["geogram"]

        if self.settings.os in ("Linux", "FreeBSD"):
            self.cpp_info.system_libs = ["m", "dl", "pthread"]
        elif self.settings.os == "Macos":
            self.cpp_info.frameworks = ["Cocoa", "IOKit"]
