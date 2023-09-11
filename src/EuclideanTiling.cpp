/**
 * @file        EuclideanTiling.cpp
 * @brief       Manage the exist and entry of particles on the border of the UV domain based on the euclidean tiling
 *
 * @author      Jan-Piotraschke
 * @date        2023-Aug-30
 * @version     0.1.0
 * @license     Apache License 2.0
 *
 * @bug         fix processPoints() because sometimes the mapping does not work and ends in an infinite loop
 * @todo        generalize process_points() due to the fact, that we need the entry_point coordinates if the particle leaves a more complex 2D mesh form, because then the old start and the new_point intersect multiple borders
 */

#include <EuclideanTiling.h>

EuclideanTiling::EuclideanTiling(
    SurfaceParametrization& surface_parametrization,
    Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV,
    Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV_old,
    Eigen::VectorXi& n
)
    : surface_parametrization(surface_parametrization),
      r_UV(r_UV),
      r_UV_old(r_UV_old),
      n(n),
      original_mesh(true)
{

}


// ========================================
// Public Functions
// ========================================

/**
 * @brief Because we have a mod(2) seam edge cute line, pairing edges are on the exact same opposite position in the UV mesh with the same lenght
*/
void EuclideanTiling::opposite_seam_edges_square_border(){
    r_UV.col(0) = r_UV.col(0).array() - r_UV.col(0).array().floor();  // Wrap x values
    r_UV.col(1) = r_UV.col(1).array() - r_UV.col(1).array().floor();  // Wrap y values
}


/**
 * @brief By using the '&' we pass the reference of the variable to the function, so we can change the value of the variable inside the function
*/
void EuclideanTiling::diagonal_seam_edges_square_border(){
    bool valid;
    do {
        valid = true;
        for (int i = 0; i < r_UV_old.rows(); ++i) {
            Eigen::Vector2d pointA = r_UV_old.row(i).head<2>(); // only takes the first two columns for the ith row
            Eigen::Vector2d point_outside = r_UV.row(i).head<2>(); // only takes the first two columns for the ith row
            double n_double = n(i);

            auto results = processPoints(pointA, point_outside, n_double);
            Eigen::Vector2d new_point = std::get<0>(results);
            n(i) = std::get<1>(results);

            // Check, wether the new point is inside the boundaries
            if (surface_parametrization.check_point_in_polygon(new_point, original_mesh)) {
                r_UV.row(i).head<2>().noalias() = new_point;
            } else {
                r_UV.row(i).head<2>().noalias() = new_point;
                valid = false;
                break;
            }
        }
    } while (!valid);
}



// ========================================
// Private Functions
// ========================================

std::pair<Eigen::Vector2d, double> EuclideanTiling::processPoints(
    const Eigen::Vector2d& pointA,
    const Eigen::Vector2d& point_outside,
    double n
) {
    Eigen::Vector2d new_point(2, 1);

    // Check, wether the point is outside the boundaries
    if (!surface_parametrization.check_point_in_polygon(point_outside, true)) {
        auto crossed_border = surface_parametrization.check_border_crossings(pointA, point_outside);
        if (crossed_border == "left") {
            new_point = Eigen::Vector2d(point_outside[1], -point_outside[0]);
            n -= KACHEL_ROTATION;
        } else if (crossed_border == "right") {
            new_point = Eigen::Vector2d(point_outside[1], 2-point_outside[0]);
            n -= KACHEL_ROTATION;
        } else if (crossed_border == "up") {
            new_point = Eigen::Vector2d(2-point_outside[1], point_outside[0]);
            n += KACHEL_ROTATION;
        } else {
            new_point = Eigen::Vector2d(-point_outside[1], point_outside[0]);
            n += KACHEL_ROTATION;
        }
    } else {
        new_point = point_outside;
    }

    return {new_point, n};
}
