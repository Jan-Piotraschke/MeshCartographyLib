---
output_filename: "SpectreMonotileHelper"

brief: "Create an optimizable border of the spectre monotile"
---

# Spectre Monotile Helper

The purpose of this module is to provide functions for creating the border of the spectre monotile.

```cpp
#include "SpectreMonotileHelper.h"

// ! NOTE: just think of "T" as "double", because that's what it is in the end
template <typename T>
std::pair<std::vector<T>, std::vector<T>> calculate_control_points(const std::pair<T, T>& point, T curve_strength)
{
    T x = point.first, y = point.second;
    T normal_x = y, normal_y = -x;
    std::vector<T> control1 = {-curve_strength * normal_x + x / T(2), -curve_strength * normal_y + y / T(2)};
    std::vector<T> control2 = {curve_strength * normal_x + x / T(2), curve_strength * normal_y + y / T(2)};
    return {control1, control2};
}

template <typename T>
void spectre_border(T a, T b, T curve_strength, std::vector<T>& x_vals, std::vector<T>& y_vals)
{
    T cos_angle = ceres::cos(T(M_PI) / T(3));
    T sin_angle = ceres::sin(T(M_PI) / T(3));

    std::vector<std::pair<T, T>> direction_vectors = {
        {cos_angle * b, sin_angle * b},
        {b, T(0)},
        {T(0), a},
        {sin_angle * a, cos_angle * a},
        {cos_angle * b, -sin_angle * b},
        {-cos_angle * b, -sin_angle * b},
        {sin_angle * a, -cos_angle * a},
        {T(0), -a},
        {T(0), -a},
        {-sin_angle * a, -cos_angle * a},
        {-cos_angle * b, sin_angle * b},
        {-b, T(0)},
        {T(0), a},
        {-sin_angle * a, cos_angle * a},
    };

    x_vals.push_back(T(0));
    y_vals.push_back(T(0));

    for (const auto& [dx, dy] : direction_vectors)
    {
        auto [control1, control2] = calculate_control_points<T>({dx, dy}, curve_strength);
        std::vector<T> P0 = {x_vals.back(), y_vals.back()};
        std::vector<T> P1 = {control1[0] + x_vals.back(), control1[1] + y_vals.back()};
        std::vector<T> P2 = {control2[0] + x_vals.back(), control2[1] + y_vals.back()};
        std::vector<T> P3 = {dx + x_vals.back(), dy + y_vals.back()};

        for (double t = 0; t <= 1; t += 0.02)
        {
            T T_t = T(t);
            T mt = T(1) - T_t;
            T mt2 = mt * mt;
            T t2 = T_t * T_t;

            T x = mt2 * mt * P0[0] + T(3) * mt2 * T_t * P1[0] + T(3) * mt * t2 * P2[0] + t2 * T_t * P3[0];
            T y = mt2 * mt * P0[1] + T(3) * mt2 * T_t * P1[1] + T(3) * mt * t2 * P2[1] + t2 * T_t * P3[1];

            x_vals.push_back(x);
            y_vals.push_back(y);
        }
    }
}

// Explicit template instantiation
template void spectre_border<double>(
    double a, double b, double curve_strength, std::vector<double>& x_vals, std::vector<double>& y_vals);
template void spectre_border<ceres::Jet<double, 1>>(
    ceres::Jet<double, 1> a,
    ceres::Jet<double, 1> b,
    ceres::Jet<double, 1> curve_strength,
    std::vector<ceres::Jet<double, 1>>& x_vals,
    std::vector<ceres::Jet<double, 1>>& y_vals);

void spectre_border_wrapper(double a, double b, double curve_strength, std::vector<double>& x_vals, std::vector<double>& y_vals) {
    spectre_border<double>(a, b, curve_strength, x_vals, y_vals);
}
```

## Draw Spectre Border and Save as Image

```cpp
std::string drawSpectreBorder(
    const std::string& filename, const std::vector<double>& x_vals, const std::vector<double>& y_vals)
{
    std::string folder = "img";
    std::filesystem::path folder_path(folder);
    if (!std::filesystem::exists(folder_path))
    {
        std::filesystem::create_directories(folder_path);
    }

    // Construct the full path
    std::filesystem::path file_path = folder_path / filename;

    // Create a white image
    int image_size = 500; // You can adjust this to the resolution you want
    cv::Mat image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255)); // Set background to white

    // Ensure the x_vals and y_vals are of the same size
    if (x_vals.size() != y_vals.size())
    {
        std::cerr << "Error: x_vals and y_vals must have the same number of points." << std::endl;
        return ""; // Return empty string on error
    }

    // Find the minimum and maximum values
    double min_x = *std::min_element(x_vals.begin(), x_vals.end());
    double max_x = *std::max_element(x_vals.begin(), x_vals.end());
    double min_y = *std::min_element(y_vals.begin(), y_vals.end());
    double max_y = *std::max_element(y_vals.begin(), y_vals.end());

    // Calculate the scaling factor
    double scale_x = (image_size * 0.8) / (max_x - min_x); // Leave some margin (80% of the image size)
    double scale_y = (image_size * 0.8) / (max_y - min_y);
    double scale = std::min(scale_x, scale_y); // Use the smaller scale to fit both dimensions

    // Calculate the offset to center the shape in the image
    double offset_x = -min_x * scale + (image_size / 2 - ((max_x - min_x) * scale) / 2);
    double offset_y = -min_y * scale + (image_size / 2 - ((max_y - min_y) * scale) / 2);

    // Loop through the border points and draw lines connecting them
    for (size_t i = 0; i < x_vals.size() - 1; ++i)
    {
        // Apply scaling and translation (offset)
        cv::Point pt1(offset_x + scale * x_vals[i], offset_y - scale * y_vals[i]); // Convert to pixel coordinates
        cv::Point pt2(offset_x + scale * x_vals[i + 1], offset_y - scale * y_vals[i + 1]); // Convert next point to
                                                                                           // pixel coordinates

        // Draw the line
        cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 2); // Draw a black line
    }

    // Optionally, connect the last point to the first to close the shape
    if (!x_vals.empty())
    {
        cv::Point pt1(offset_x + scale * x_vals.back(), offset_y - scale * y_vals.back());
        cv::Point pt2(offset_x + scale * x_vals.front(), offset_y - scale * y_vals.front());
        cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 2); // Close the shape with a black line
    }

    // Save the image
    bool success = cv::imwrite(file_path.string(), image);
    if (!success)
    {
        std::cerr << "Error saving image to " << file_path << std::endl;
        return ""; // Return empty string on error
    }
    else
    {
        std::cout << "Spectre border saved to " << file_path << std::endl;
        return file_path.string(); // Return the path to the saved image
    }
}
```
