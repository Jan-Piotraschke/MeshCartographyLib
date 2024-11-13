#include <pybind11/pybind11.h>
#include "mesh_cartography.h"

namespace py = pybind11;

PYBIND11_MODULE(mesh_cartography_module, m) {
    m.def("run_mesh_cartography", &run_mesh_cartography, py::arg("free_boundary") = false,
          "Runs the mesh cartography process with the specified boundary condition.");
}
