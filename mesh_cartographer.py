import sys
import os

# Adjust the path to the directory containing the module
module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'python'))
sys.path.append(module_path)

import mesh_cartography_module

free_boundary = False  # or True
mesh_cartography_module.run_mesh_cartography(free_boundary)
