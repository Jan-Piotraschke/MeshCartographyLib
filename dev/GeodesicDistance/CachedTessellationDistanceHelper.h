#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>

#include "GeodesicDistanceHelperInterface.h"
#include "TessellationDistanceHelper.h"

class CachedTessellationDistanceHelper : public GeodesicDistanceHelperInterface
{
  public:
    CachedTessellationDistanceHelper(fs::path mesh_path, std::vector<std::vector<int64_t>> equivalent_vertices);

    Eigen::MatrixXd get_mesh_distance_matrix() override;

  private:
    fs::path mesh_path;
    std::vector<std::vector<int64_t>> equivalent_vertices;
    TessellationDistanceHelper geodesic_distance_helper;

    template <typename MatrixType>
    void save_csv(const MatrixType& distance_matrix_v, fs::path cache_file)
    {
        std::cout << "Saving distance matrix to file..." << std::endl;
        std::ofstream file(cache_file.string());

        // Iterate over the lower triangular part and write to the file.
        for (int i = 0; i < distance_matrix_v.rows(); ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                file << distance_matrix_v(i, j);
                if (j != i)
                {
                    file << ", ";
                }
            }
            file << "\n";
        }

        file.close();
        std::cout << "saved" << std::endl;
    }

    template <typename MatrixType>
    MatrixType load_csv(fs::path cache_file)
    {
        std::ifstream indata;
        indata.open(cache_file.string());
        std::string line;
        std::vector<std::vector<double>> values;

        while (std::getline(indata, line))
        {
            std::stringstream lineStream(line);
            std::string cell;
            std::vector<double> rowValues;
            while (std::getline(lineStream, cell, ','))
            {
                rowValues.push_back(std::stod(cell));
            }
            values.push_back(rowValues);
        }

        // The number of rows in the full symmetric matrix will be the size of values
        int matrix_size = values.size();
        MatrixType symmetric_matrix(matrix_size, matrix_size);
        symmetric_matrix.setZero();

        // Fill the symmetric matrix using the triangular data
        for (int i = 0; i < matrix_size; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                symmetric_matrix(i, j) = values[i][j];
                symmetric_matrix(j, i) = values[i][j];
            }
        }

        return symmetric_matrix;
    }
};
