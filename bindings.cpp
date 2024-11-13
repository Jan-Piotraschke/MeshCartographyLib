#include <pybind11/pybind11.h>
#include "mesh_cartography.h"
#include "SurfaceParametrization/SurfaceParametrization.h"

namespace py = pybind11;

PYBIND11_MODULE(mesh_cartography_module, m) {
    m.def("run_mesh_cartography", &run_mesh_cartography, py::arg("free_boundary") = false,
          "Runs the mesh cartography process with the specified boundary condition and returns the monotile area.");

    // Class binding: SurfaceParametrization
    py::class_<SurfaceParametrization>(m, "SurfaceParametrization")
        .def(py::init<>())
        .def("create_uv_surface", &SurfaceParametrization::create_uv_surface)
        .def("get_mesh_name", &SurfaceParametrization::get_mesh_name);
}
