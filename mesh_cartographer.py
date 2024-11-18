import os
import sys
import json
from pathlib import Path

# Adjust the path to the directory containing the module
module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "python"))
sys.path.append(module_path)

import mesh_cartography_module as mc


def main():
    # Initialize parameters for mesh cartography
    free_boundary = False
    initial_area = mc.run_mesh_cartography(free_boundary)
    print(f"Initial Area: {initial_area}")

    # Using the SurfaceParametrization class
    sp = mc.SurfaceParametrization()
    # uv_mesh = sp.create_uv_surface("./meshes/ellipsoid_x4.off", 0)
    # mesh_name = sp.get_mesh_name(uv_mesh[3])

    # Parameters for border generation
    a = 1.0
    b = 1.0
    curve_strength = 1.0

    # Generate the border points
    x_vals, y_vals = mc.spectre_border(a, b, curve_strength)

    # Draw the border and get the image path
    image_filename = "spectre_border.png"
    image_path = mc.drawSpectreBorder(image_filename, x_vals, y_vals)
    print(f"Spectre border image saved at: {image_path}")

    # Save data to JSON file
    data = {
        "initial_area": initial_area,
    }
    data_filename = "data.json"
    with open(data_filename, "w") as f:
        json.dump(data, f)
    print(f"Data saved to {data_filename}")


if __name__ == "__main__":
    main()
