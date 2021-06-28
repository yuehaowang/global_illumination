#pragma once
#include <vector>
#include <Eigen/Dense>


bool loadObj(
    const char* path,
    std::vector <Eigen::Vector3f>& out_vertices,
    std::vector <Eigen::Vector2f>& out_uvs,
    std::vector <Eigen::Vector3f>& out_normals,
    std::vector <int>& out_v_index,
    std::vector <int>& out_vt_index,
    std::vector <int>& out_vn_index
);

