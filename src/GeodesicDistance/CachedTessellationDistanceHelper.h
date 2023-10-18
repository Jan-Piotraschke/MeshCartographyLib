#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>

#include "GeodesicDistanceHelperInterface.h"
#include "TessellationDistanceHelper.h"


class CachedTessellationDistanceHelper : public GeodesicDistanceHelperInterface {
public:
    CachedTessellationDistanceHelper(fs::path mesh_path, std::vector<std::vector<int64_t>> equivalent_vertices);

    Eigen::MatrixXd get_mesh_distance_matrix() override;

private:
    fs::path mesh_path;
    std::vector<std::vector<int64_t>> equivalent_vertices;
    TessellationDistanceHelper geodesic_distance_helper;

    template<typename MatrixType>
    void save_csv(MatrixType& distance_matrix_v, fs::path cache_file) {

        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

        std::cout << "Saving distance matrix to file..." << std::endl;
        std::ofstream file(cache_file.string());
        file << distance_matrix_v.format(CSVFormat);
        file.close();
        std::cout << "saved" << std::endl;
    };

    template<typename MatrixType>
    MatrixType load_csv(fs::path cache_file) {
        std::ifstream indata;
        indata.open(cache_file.string());
        std::string line;
        std::vector<double> values;
        uint rows = 0;
        while (std::getline(indata, line)) {
            std::stringstream lineStream(line);
            std::string cell;
            while (std::getline(lineStream, cell, ',')) {
                values.push_back(std::stod(cell));
            }
            ++rows;
        }
        return Eigen::Map<const Eigen::Matrix<typename MatrixType::Scalar, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
    }
};
