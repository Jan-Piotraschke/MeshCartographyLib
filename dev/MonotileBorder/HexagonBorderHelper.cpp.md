---
output_filename: "HexagonBorderHelper"

brief: "Create a hexagonal border"
---

# Hexagon Border Helper

The purpose of this module is to provide functions for creating a hexagonal border for an UV mesh model.

```cpp
#include "HexagonBorderHelper.h"
#include <CGAL/squared_distance_2.h> // For distance computation
#include <cmath>                     // For M_PI
#include <limits>                    // For std::numeric_limits
#include <opencv2/opencv.hpp>        // OpenCV for drawing and saving the image

HexagonBorderHelper::HexagonBorderHelper(UV::Mesh& mesh, UV::halfedge_descriptor bhd, _3D::UV_pmap& uvmap)
    : mesh(mesh), bhd(bhd), uvmap(uvmap)
{
}

void HexagonBorderHelper::setup_hexagon_boundary_constraints()
{
    // Collect boundary loop
    std::vector<UV::vertex_descriptor> boundary_loop;
    auto h = bhd;
    do
    {
        boundary_loop.push_back(target(h, mesh));
        h = next(h, mesh);
    } while (h != bhd);

    unsigned int N = boundary_loop.size();

    // Ensure N is divisible by 6
    N = N - (N % 6); // Truncate N to be divisible by 6
    if (N == 0)
    {
        std::cerr << "Error: No valid vertices in boundary loop after truncation." << std::endl;
        return;
    }

    // Only use the first N vertices for the UV mapping
    std::vector<UV::vertex_descriptor> truncated_boundary_loop(boundary_loop.begin(), boundary_loop.begin() + N);

    // Define the angles for the hexagon vertices
    double hex_angle_increment = M_PI / 3; // 60 degrees per side
    double hex_radius = 1.0;               // Arbitrary radius for the hexagon in UV space

    // Number of vertices per side of the hexagon
    unsigned int vertices_per_side = N / 6;

    // Calculate UV coordinates for boundary vertices along the hexagon perimeter
    for (unsigned int i = 0; i < N; ++i)
    {
        // Compute which side of the hexagon the vertex belongs to
        int hex_side = i / vertices_per_side;                              // Determine the hexagon side
        double frac = (double)(i % vertices_per_side) / vertices_per_side; // Fraction along that side

        // Compute the start and end of the current hexagon side
        double start_angle = hex_side * hex_angle_increment;
        double end_angle = (hex_side + 1) * hex_angle_increment;

        // Compute the UV coordinates by interpolating between the two points on the hexagon
        double start_x = hex_radius * std::cos(start_angle);
        double start_y = hex_radius * std::sin(start_angle);
        double end_x = hex_radius * std::cos(end_angle);
        double end_y = hex_radius * std::sin(end_angle);

        double uv_x = start_x * (1.0 - frac) + end_x * frac;
        double uv_y = start_y * (1.0 - frac) + end_y * frac;

        // Assign the calculated UV coordinates to the vertex
        put(uvmap, truncated_boundary_loop[i], Point_2(uv_x, uv_y));
    }
}
```

## Draw Hexagon and Save as Image

```cpp
void HexagonBorderHelper::drawHexagon(const std::string& filename)
{
    // Create a white image
    int image_size = 500;
    cv::Mat image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));

    // Set up scaling and centering
    double scale = 200.0;                             // Scaling factor for the UV coordinates
    cv::Point center(image_size / 2, image_size / 2); // Center of the image

    // Collect boundary loop
    std::vector<UV::vertex_descriptor> boundary_loop;
    auto h = bhd;
    do
    {
        boundary_loop.push_back(target(h, mesh));
        h = next(h, mesh);
    } while (h != bhd);

    unsigned int N = boundary_loop.size();

    // Truncate N to ensure it is divisible by 6
    N = N - (N % 6);
    std::vector<UV::vertex_descriptor> truncated_boundary_loop(boundary_loop.begin(), boundary_loop.begin() + N);

    // Loop through the boundary vertices and draw lines connecting them based on UV coordinates
    for (size_t i = 0; i < truncated_boundary_loop.size(); ++i)
    {
        // Get UV coordinates of the current vertex
        Point_2 uv1 = get(uvmap, truncated_boundary_loop[i]);
        Point_2 uv2 = get(uvmap, truncated_boundary_loop[(i + 1) % truncated_boundary_loop.size()]); // Wrap around

        // Convert UV coordinates to pixel coordinates in the image
        cv::Point pt1(center.x + scale * uv1.x(), center.y + scale * uv1.y());
        cv::Point pt2(center.x + scale * uv2.x(), center.y + scale * uv2.y());

        // Draw the line
        cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 2); // Black lines
    }

    // Save the image
    bool success = cv::imwrite(filename, image);
    if (!success)
    {
        std::cerr << "Error saving image to " << filename << std::endl;
    }
    else
    {
        std::cout << "Hexagon saved to " << filename << std::endl;
    }
}
```

## Initialize Hexagon Corners and Map to Hexagon

```cpp
void HexagonBorderHelper::initializeCorners(double sideLength)
{
    corners.clear();
    double angle_increment = M_PI / 3; // 60 degrees in radians for hexagon

    // Initialize the hexagon corners relative to the center (0, 0)
    for (int i = 0; i < 6; ++i)
    {
        // Calculate x and y using polar coordinates
        double angle = i * angle_increment;
        double x = sideLength * std::cos(angle);
        double y = sideLength * std::sin(angle);
        corners.push_back(Corner(Eigen::Vector2d(x, y), sideLength));
    }
}

Point_2 HexagonBorderHelper::mapToHexagon(double l)
{
    double sum = 0.0;
    for (size_t i = 0; i < corners.size(); ++i)
    {
        const auto& corner = corners[i];
        const auto& nextCorner = corners[(i + 1) % corners.size()];
        if (l <= sum + corner.sideLength)
        {
            double frac = (l - sum) / corner.sideLength;
            Eigen::Vector2d pos = corner.position * (1.0 - frac) + nextCorner.position * frac;
            return Point_2(pos.x(), pos.y());
        }
        sum += corner.sideLength;
    }
    return Point_2(0.0, 0.0);
}
```
