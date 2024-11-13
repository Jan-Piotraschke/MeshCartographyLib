import sys
import os

# Adjust the path to the directory containing the module
module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "python"))
sys.path.append(module_path)

import mesh_cartography_module as mc

free_boundary = False
initial_area = mc.run_mesh_cartography(free_boundary)
print(f"Initial Area: {initial_area}")

# Using the SurfaceParametrization class
sp = mc.SurfaceParametrization()
uv_mesh = sp.create_uv_surface("./meshes/ellipsoid_x4.off", 0)
# mesh_name = sp.get_mesh_name(uv_mesh[3])
# print(f"Mesh name: {mesh_name}")
