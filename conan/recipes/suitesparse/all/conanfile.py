from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import get


class SuiteSparseConan(ConanFile):
    name = "suitesparse"
    version = "7.10.3"
    license = "BSD-3-Clause AND LGPL-2.1-or-later AND GPL-2.0-or-later"
    url = "https://github.com/DrTimothyAldenDavis/SuiteSparse"
    description = "A suite of sparse matrix algorithms"
    topics = ("sparse-matrix", "linear-algebra", "numerical")

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

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
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["BUILD_TESTING"] = False
        tc.variables["BUILD_SHARED_LIBS"] = bool(self.options.shared)
        tc.variables["SUITESPARSE_USE_CUDA"] = False
        tc.variables["SUITESPARSE_DEMOS"] = False
        tc.variables["SUITESPARSE_ENABLE_PROJECTS"] = (
            "suitesparse_config;amd;camd;ccolamd;colamd;"
            "cholmod;cxsparse;klu;umfpack;spqr"
        )
        tc.variables["SUITESPARSE_USE_FORTRAN"] = False
        tc.variables["SUITESPARSE_USE_OPENMP"] = False
        tc.variables["BLA_VENDOR"] = ""
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "SuiteSparse")

        # SuiteSparse_config
        self.cpp_info.components["suitesparse_config"].set_property(
            "cmake_target_name", "SuiteSparse::SuiteSparseConfig"
        )
        self.cpp_info.components["suitesparse_config"].libs = ["suitesparseconfig"]

        # AMD
        self.cpp_info.components["amd"].set_property(
            "cmake_target_name", "SuiteSparse::AMD"
        )
        self.cpp_info.components["amd"].libs = ["amd"]
        self.cpp_info.components["amd"].requires = ["suitesparse_config"]

        # CAMD
        self.cpp_info.components["camd"].set_property(
            "cmake_target_name", "SuiteSparse::CAMD"
        )
        self.cpp_info.components["camd"].libs = ["camd"]
        self.cpp_info.components["camd"].requires = ["suitesparse_config"]

        # CCOLAMD
        self.cpp_info.components["ccolamd"].set_property(
            "cmake_target_name", "SuiteSparse::CCOLAMD"
        )
        self.cpp_info.components["ccolamd"].libs = ["ccolamd"]
        self.cpp_info.components["ccolamd"].requires = ["suitesparse_config"]

        # COLAMD
        self.cpp_info.components["colamd"].set_property(
            "cmake_target_name", "SuiteSparse::COLAMD"
        )
        self.cpp_info.components["colamd"].libs = ["colamd"]
        self.cpp_info.components["colamd"].requires = ["suitesparse_config"]

        # CHOLMOD
        self.cpp_info.components["cholmod"].set_property(
            "cmake_target_name", "SuiteSparse::CHOLMOD"
        )
        self.cpp_info.components["cholmod"].libs = ["cholmod"]
        self.cpp_info.components["cholmod"].requires = [
            "suitesparse_config", "amd", "camd", "colamd", "ccolamd",
        ]

        # CXSparse
        self.cpp_info.components["cxsparse"].set_property(
            "cmake_target_name", "SuiteSparse::CXSparse"
        )
        self.cpp_info.components["cxsparse"].libs = ["cxsparse"]
        self.cpp_info.components["cxsparse"].requires = ["suitesparse_config"]

        # KLU
        self.cpp_info.components["klu"].set_property(
            "cmake_target_name", "SuiteSparse::KLU"
        )
        self.cpp_info.components["klu"].libs = ["klu"]
        self.cpp_info.components["klu"].requires = [
            "suitesparse_config", "amd", "colamd",
        ]

        # UMFPACK
        self.cpp_info.components["umfpack"].set_property(
            "cmake_target_name", "SuiteSparse::UMFPACK"
        )
        self.cpp_info.components["umfpack"].libs = ["umfpack"]
        self.cpp_info.components["umfpack"].requires = [
            "suitesparse_config", "amd", "cholmod",
        ]

        # SPQR
        self.cpp_info.components["spqr"].set_property(
            "cmake_target_name", "SuiteSparse::SPQR"
        )
        self.cpp_info.components["spqr"].libs = ["spqr"]
        self.cpp_info.components["spqr"].requires = [
            "suitesparse_config", "cholmod",
        ]

        if self.settings.os in ("Linux", "FreeBSD"):
            self.cpp_info.components["suitesparse_config"].system_libs = ["m"]
