#pragma once
#include <utility>
#include <cmath>
#include <vector>
#include "ray.hpp"


/**
 * Axis-Aligned Bounding Box
 */

struct AABB {
    Eigen::Vector3f lb;
    Eigen::Vector3f ub;

    AABB();
    /* Construct AABB by coordinates of lower bound and upper bound */
    AABB(float lb_x, float lb_y, float lb_z, float ub_x, float ub_y, float ub_z);
    AABB(Eigen::Vector3f lb, Eigen::Vector3f ub);
    /* Construct AABB for a sphere */
    AABB(const Eigen::Vector3f& pos, float radius);
    /* Construct AABB for a triangle */
    AABB(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3);
    /* Construct AABB by merging two AABBs */
    AABB(const AABB& a, const AABB& b);
    /* Get the AABB center */
    Eigen::Vector3f getCenter() const;
    /* Get the length of a specified side on the AABB */
    float getDist(int c) const;
    /* Get the volume of the AABB */
    float getVolume() const;
    /* Check whether the AABB is overlapping with another AABB */
    bool isOverlap(const AABB& a) const;
    /* Get the diagonal length */
    float diagonalLength() const;
    /* Test intersection with a ray */
    bool rayIntersection(const Ray& ray, float& t_in, float& t_out) const;
};


/**
 * Uniform Grid
 */

typedef std::vector<Eigen::Vector3i> Cell;
typedef std::vector<Cell> Cells;

struct Grid
{
    Eigen::Vector3i N;
    float cell_size;
    AABB bounding_box;
    Cells cells;

    Grid();
    /* Initialize the uniform grid */
    void create(const AABB& bb, unsigned int N, float lambda = 3);
    /* Retrieve the cell at (x,y,z) */
    Cell* getCell(unsigned int x, unsigned int y, unsigned int z);
    /* Add one triangle to a cell */
    void addTriangle(Eigen::Vector3i tri_indices, const AABB& tri_bb);
    /* Reset the grid */
    void reset();
};
