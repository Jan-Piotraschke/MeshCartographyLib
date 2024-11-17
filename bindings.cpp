#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "mesh_cartography.h"
#include "SurfaceParametrization/SurfaceParametrization.h"
#include "MonotileBorder/SpectreMonotileHelper.h"

namespace py = pybind11;

// Function to generate spectre border and return x_vals and y_vals
std::pair<std::vector<double>, std::vector<double>> generate_spectre_border(double a, double b, double curve_strength) {
    std::vector<double> x_vals, y_vals;
    spectre_border_wrapper(a, b, curve_strength, x_vals, y_vals);
    return std::make_pair(x_vals, y_vals);
}

PYBIND11_MODULE(mesh_cartography_module, m) {
    m.def("run_mesh_cartography", &run_mesh_cartography, py::arg("free_boundary") = false,
          "Runs the mesh cartography process with the specified boundary condition and returns the monotile area.");

    // Class binding: SurfaceParametrization
    py::class_<SurfaceParametrization>(m, "SurfaceParametrization")
        .def(py::init<>())
        .def("create_uv_surface", &SurfaceParametrization::create_uv_surface)
        .def("get_mesh_name", &SurfaceParametrization::get_mesh_name);

    // Expose spectre_border function
    m.def("spectre_border", &generate_spectre_border, py::arg("a"), py::arg("b"), py::arg("curve_strength"),
          "Generates the spectre border points and returns (x_vals, y_vals).");

    // Expose drawSpectreBorder function
    m.def("drawSpectreBorder", &drawSpectreBorder, py::arg("filename"), py::arg("x_vals"), py::arg("y_vals"),
          "Draws the spectre border and saves the image, returns the image path.");
}
