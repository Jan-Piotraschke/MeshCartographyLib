import sys
import os
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas

# Adjust the path to the directory containing the module
module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "python"))
sys.path.append(module_path)

import mesh_cartography_module as mc

free_boundary = False
initial_area = mc.run_mesh_cartography(free_boundary)
print(f"Initial Area: {initial_area}")

# Using the SurfaceParametrization class
sp = mc.SurfaceParametrization()
# uv_mesh = sp.create_uv_surface("./meshes/ellipsoid_x4.off", 0)
# mesh_name = sp.get_mesh_name(uv_mesh[3])
# print(f"Mesh name: {mesh_name}")

# Parameters
a = 1.0
b = 1.0
curve_strength = 1.0

# Generate the border points
x_vals, y_vals = mc.spectre_border(a, b, curve_strength)

# Draw the border and get the image path
image_path = mc.drawSpectreBorder("spectre_border.png", x_vals, y_vals)

print(f"Python: Spectre border image saved at: {image_path}")

# Create the 'pdf' directory if it doesn't exist
pdf_folder = "pdf"
if not os.path.exists(pdf_folder):
    os.makedirs(pdf_folder)

# Define the path for the PDF file
pdf_filename = "mesh_cartographer.pdf"
pdf_path = os.path.join(pdf_folder, pdf_filename)


# Create a PDF and add the image
def create_pdf_with_image(image_path, pdf_path):
    # Create a canvas object
    c = canvas.Canvas(pdf_path, pagesize=letter)
    width, height = letter

    # Draw the image at full page size
    c.drawImage(image_path, 0, 0, width=width, height=height)

    # Save the PDF file
    c.save()
    print(f"PDF saved at: {pdf_path}")


# Call the function to create the PDF
try:
    create_pdf_with_image(image_path, pdf_path)
except Exception as e:
    print(f"An error occurred while creating the PDF: {e}", file=sys.stderr)
