#include "HexagonBorderHelper.h"
#include <CGAL/squared_distance_2.h>  // For distance computation
#include <opencv2/opencv.hpp>  // OpenCV for drawing and saving the image
#include <cmath>               // For M_PI

// ========================================
// Constructor
// ========================================

HexagonBorderHelper::HexagonBorderHelper(
    UV::Mesh& mesh,
    UV::halfedge_descriptor bhd,
    _3D::UV_pmap& uvmap
)
    : mesh(mesh),
    bhd(bhd),
    uvmap(uvmap)
{}

// ========================================
// Public Functions
// ========================================

void HexagonBorderHelper::setup_hexagon_boundary_constraints()
{
    // Initialize all texture coordinates to the origin
    for (auto v : vertices(mesh)) {
        put(uvmap, v, Point_2(0.0, 0.0));  // Initialize all to (0, 0)
    }

    // Collect boundary loop
    std::vector<UV::vertex_descriptor> boundary_loop;  // Corrected type
    auto h = bhd;
    do {
        boundary_loop.push_back(target(h, mesh));  // Corrected type and method
        h = next(h, mesh);
    } while (h != bhd);

    unsigned int N = boundary_loop.size();
    double loop_length = 0.0;

    // Compute length of the boundary loop
    for (unsigned int i = 0; i < N; ++i) {
        loop_length += CGAL::sqrt(CGAL::squared_distance(
            get(uvmap, boundary_loop[i]),
            get(uvmap, boundary_loop[(i + 1) % N])
        ));
    }

    // Log the computed loop length
    std::cout << "Computed boundary loop length: " << loop_length << std::endl;

    // Initialize the hexagon corners
    double sideLength = loop_length / 6.0;

    // Log the side length for debugging
    std::cout << "Computed hexagon side length: " << sideLength << std::endl;

    initializeCorners(sideLength);

    double step_size = loop_length / N;
    double l = 0.0;
    auto tolerance = 1e-4;

    // Map the length intervals to hexagon intervals
    for (unsigned int i = 0; i < N; ++i, l += step_size) {
        Point_2 t = mapToHexagon(l);

        // Apply tolerance
        if (std::abs(t.x()) < tolerance) t = Point_2(0.0, t.y());
        if (std::abs(t.y()) < tolerance) t = Point_2(t.x(), 0.0);

        // Assign texture coordinate
        put(uvmap, boundary_loop[i], t);
    }
}


// ========================================
// Function to draw the hexagon and save it as an image
// ========================================

void HexagonBorderHelper::drawHexagon(const std::string& filename)
{
    // Create a white image
    int image_size = 500;
    cv::Mat image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));  // White background

    // Log corner positions for debugging
    std::cout << "Hexagon Corners:" << std::endl;
    for (size_t i = 0; i < corners.size(); ++i) {
        std::cout << "Corner " << i << ": (" << corners[i].position.x() << ", " << corners[i].position.y() << ")" << std::endl;
    }

    // Calculate the bounding box of the hexagon to adjust the scaling factor dynamically
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& corner : corners) {
        min_x = std::min(min_x, corner.position.x());
        max_x = std::max(max_x, corner.position.x());
        min_y = std::min(min_y, corner.position.y());
        max_y = std::max(max_y, corner.position.y());
    }

    // Calculate the scale based on the size of the bounding box
    double scale_x = (image_size - 50) / (max_x - min_x);  // 50 px padding
    double scale_y = (image_size - 50) / (max_y - min_y);  // 50 px padding
    double scale = std::min(scale_x, scale_y);  // Use the smaller scale to fit the hexagon

    // Center the hexagon within the image
    cv::Point center(image_size / 2, image_size / 2);

    // Loop through the corners and draw lines connecting them
    for (size_t i = 0; i < corners.size(); ++i) {
        const auto& current_corner = corners[i];
        const auto& next_corner = corners[(i + 1) % corners.size()];

        cv::Point pt1(center.x + scale * (current_corner.position.x() - min_x),
                      center.y + scale * (current_corner.position.y() - min_y));
        cv::Point pt2(center.x + scale * (next_corner.position.x() - min_x),
                      center.y + scale * (next_corner.position.y() - min_y));

        cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 2);  // Black lines
    }

    // Save the image
    cv::imwrite(filename, image);

    std::cout << "Hexagon saved to " << filename << std::endl;
}




// ========================================
// Private Functions
// ========================================

void HexagonBorderHelper::initializeCorners(double sideLength)
{
    corners.clear();
    double angle_increment = M_PI / 3;  // 60 degrees in radians for hexagon

    // Initialize the hexagon corners relative to the center (0, 0)
    for (int i = 0; i < 6; ++i) {
        // Calculate x and y using polar coordinates
        double angle = i * angle_increment;
        double x = sideLength * std::cos(angle);  // X-coordinate of the corner
        double y = sideLength * std::sin(angle);  // Y-coordinate of the corner
        corners.push_back(Corner(Eigen::Vector2d(x, y), sideLength));
    }
}


Point_2 HexagonBorderHelper::mapToHexagon(double l)
{
    double sum = 0.0;
    for (size_t i = 0; i < corners.size(); ++i) {
        const auto& corner = corners[i];
        const auto& nextCorner = corners[(i + 1) % corners.size()];
        if (l <= sum + corner.sideLength) {
            double frac = (l - sum) / corner.sideLength;
            Eigen::Vector2d pos = corner.position * (1.0 - frac) + nextCorner.position * frac;
            return Point_2(pos.x(), pos.y());
        }
        sum += corner.sideLength;
    }
    return Point_2(0.0, 0.0);  // Fallback, shouldn't be reached
}
