#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>

#include "pgoLogging.h"

#include "core.h"

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace nb = nanobind;

// Forward declarations of bind functions
void init_mesh_geo_bindings(nb::module_ &m);
void init_volume_mesh_bindings(nb::module_ &m);
void init_formulation_bindings(nb::module_ &m);
void init_elastic_model_bindings(nb::module_ &m);
void init_plastic_model_bindings(nb::module_ &m);
void init_sparse_bindings(nb::module_ &m);
void init_dense_bindings(nb::module_ &m);
void init_energy_bindings(nb::module_ &m);
void init_constraint_bindings(nb::module_ &m);
void init_solver_bindings(nb::module_ &m);
void init_animation_bindings(nb::module_ &m);
void init_implicit_bindings(nb::module_ &m);
void init_parallel_bindings(nb::module_ &m);
void init_simulation_bindings(nb::module_ &m);
void init_contact_bindings(nb::module_ &m);

NB_MODULE(_core, m) {
    m.attr("__version__") = VERSION_INFO;

    m.def("build_info", &pgo::buildInfo);

    m.def("set_log_level", [](const std::string &level) {
        static const std::unordered_map<std::string, spdlog::level::level_enum> levels = {
            { "trace", spdlog::level::trace },
            { "debug", spdlog::level::debug },
            { "info", spdlog::level::info },
            { "warn", spdlog::level::warn },
            { "error", spdlog::level::err },
            { "critical", spdlog::level::critical },
            { "off", spdlog::level::off },
        };
        const auto it = levels.find(level);
        if (it == levels.end())
            throw std::invalid_argument(
                "invalid log level '" + level +
                "'; expected one of trace/debug/info/warn/error/critical/off");
        pgo::Logging::setLevel(it->second);
        // The Python runtime never calls Logging::init(), so the pgo logger is
        // null and spdlog's default logger emits the messages. Set both so the
        // level applies in either case.
        spdlog::set_level(it->second);
    });
    m.def("get_log_level", []() -> std::string {
        switch (pgo::Logging::lgr()->level()) {
        case spdlog::level::trace: return "trace";
        case spdlog::level::debug: return "debug";
        case spdlog::level::info: return "info";
        case spdlog::level::warn: return "warn";
        case spdlog::level::err: return "error";
        case spdlog::level::critical: return "critical";
        case spdlog::level::off: return "off";
        }
        return "info";
    });

    init_mesh_geo_bindings(m);
    init_volume_mesh_bindings(m);
    init_formulation_bindings(m);
    init_elastic_model_bindings(m);
    init_plastic_model_bindings(m);
    init_sparse_bindings(m);
    init_dense_bindings(m);
    init_energy_bindings(m);
    init_constraint_bindings(m);
    init_solver_bindings(m);
    init_animation_bindings(m);
    init_implicit_bindings(m);
    init_parallel_bindings(m);
    init_simulation_bindings(m);
    init_contact_bindings(m);
}
