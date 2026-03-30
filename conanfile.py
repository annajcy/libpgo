from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout


class LibpgoConan(ConanFile):
    settings = "os", "arch", "compiler", "build_type"

    options = {
        "with_python": [True, False],
        "with_animation_io": [True, False],
        "with_geometry_stack": [True, False],
        "with_libigl": [True, False],
        "with_geogram": [True, False],
        "with_gmsh": [True, False],
        "with_mkl": [True, False],
        "with_arpack": [True, False],
        "profile_full": [True, False],
    }

    default_options = {
        "with_python": False,
        "with_animation_io": False,
        "with_geometry_stack": False,
        "with_libigl": False,
        "with_geogram": False,
        "with_gmsh": False,
        "with_mkl": False,
        "with_arpack": False,
        "profile_full": False,
        "fmt/*:header_only": True,
        "hwloc/*:shared": True,
        "spdlog/*:header_only": True,
        "tinyobjloader/*:double": True,
    }

    def _effective_features(self):
        profile_full = bool(self.options.profile_full)

        with_python = profile_full or bool(self.options.with_python)
        with_animation_io = profile_full or bool(self.options.with_animation_io)
        with_libigl = profile_full or bool(self.options.with_libigl)
        with_geogram = profile_full or bool(self.options.with_geogram)
        with_gmsh = profile_full or bool(self.options.with_gmsh)
        with_mkl = bool(self.options.with_mkl)
        with_arpack = bool(self.options.with_arpack)

        with_geometry_stack = (
            profile_full
            or with_python
            or with_libigl
            or with_geogram
            or bool(self.options.with_geometry_stack)
        )

        return {
            "profile_full": profile_full,
            "with_python": with_python,
            "with_animation_io": with_animation_io,
            "with_geometry_stack": with_geometry_stack,
            "with_libigl": with_libigl,
            "with_geogram": with_geogram,
            "with_gmsh": with_gmsh,
            "with_mkl": with_mkl,
            "with_arpack": with_arpack,
        }

    def build_requirements(self):
        self.tool_requires("cmake/[>=3.28 <4.0.0]")
        self.tool_requires("ninja/[>=1.11 <2.0]")

    def requirements(self):
        features = self._effective_features()

        # Core dependencies.
        self.requires("onetbb/2021.12.0")
        self.requires("eigen/3.4.0")
        self.requires("fmt/11.2.0")
        self.requires("nlohmann_json/3.11.3")
        self.requires("spdlog/1.15.3")
        self.requires("argparse/3.1")
        self.requires("tinyobjloader/2.0.0-rc10")
        self.requires("stb/cci.20240531")
        self.requires("autodiff/1.1.2")
        self.requires("tetgen/1.6.0")
        self.requires("ccd-safe/1.0.0")
        self.requires("ccd-exact/1.0.0")
        self.requires("asa/1.0.0")

        if features["with_geometry_stack"]:
            self.requires("boost/1.83.0")
            self.requires("cgal/6.0.1")
            self.requires("ceres-solver/2.2.0")
            self.requires("nlopt/2.10.0")
            self.requires("suitesparse/7.10.3")

        if features["with_python"]:
            self.requires("pybind11/2.12.0")

        if features["with_animation_io"]:
            self.requires("alembic/1.8.9")
            self.requires("imath/3.1.12")

        if features["with_libigl"]:
            self.requires("libigl/2.6.0")

        if features["with_geogram"]:
            self.requires("geogram/1.9.0")

        if features["with_gmsh"]:
            self.requires("gmsh/4.13.1")

        if features["with_mkl"]:
            self.output.warning(
                "with_mkl=ON requires private/system MKL integration; "
                "ConanCenter package is not available."
            )

        if features["with_arpack"]:
            self.requires("arpack-ng/3.9.1")

        self.test_requires("gtest/1.14.0")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        features = self._effective_features()

        tc = CMakeToolchain(self)
        tc.user_presets_path = False
        tc.variables["PGO_PROFILE_FULL"] = features["profile_full"]
        tc.variables["PGO_FEATURE_PYTHON"] = features["with_python"]
        tc.variables["PGO_FEATURE_ANIMATION_IO"] = features["with_animation_io"]
        tc.variables["PGO_FEATURE_GEOMETRY_STACK"] = features["with_geometry_stack"]
        tc.variables["PGO_FEATURE_LIBIGL"] = features["with_libigl"]
        tc.variables["PGO_FEATURE_GEOGRAM"] = features["with_geogram"]
        tc.variables["PGO_FEATURE_GMSH"] = features["with_gmsh"]
        tc.variables["PGO_FEATURE_MKL"] = features["with_mkl"]
        tc.variables["PGO_FEATURE_ARPACK"] = features["with_arpack"]
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()
