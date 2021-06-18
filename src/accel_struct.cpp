#pragma once
#include "accel_struct.hpp"


/**
 * AABB class
 */

AABB::AABB()
    : lb(0, 0, 0)
    , ub(0, 0, 0)
{
}
    
AABB::AABB(float lb_x, float lb_y, float lb_z, float ub_x, float ub_y, float ub_z)
{
    lb = Eigen::Vector3f(lb_x, lb_y, lb_z);
    ub = Eigen::Vector3f(ub_x, ub_y, ub_z);
}

AABB::AABB(Eigen::Vector3f lb, Eigen::Vector3f ub)
    : lb(lb)
    , ub(ub)
{
}

AABB::AABB(const Eigen::Vector3f& pos, float radius)
{
    Eigen::Vector3f r(radius, radius, radius);
    lb = pos - r;
    ub = pos + r;
}

AABB::AABB(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3)
{
    lb = v1.cwiseMin(v2).cwiseMin(v3);
    ub = v1.cwiseMax(v2).cwiseMax(v3);
}

AABB::AABB(const AABB& a, const AABB& b)
{
    lb = Eigen::Vector3f(a.lb.cwiseMin(b.lb));
    ub = Eigen::Vector3f(a.ub.cwiseMax(b.ub));
}

Eigen::Vector3f AABB::getCenter() const
{
    return (lb + ub) / 2;
}

float AABB::getDist(int c) const
{
    return ub[c] - lb[c];
}

float AABB::getVolume() const
{
    return getDist(2) * getDist(1) * getDist(0);
}

bool AABB::isOverlap(const AABB& a) const
{
    return ((a.lb[0] >= this->lb[0] && a.lb[0] <= this->ub[0]) || (this->lb[0] >= a.lb[0] && this->lb[0] <= a.ub[0])) &&
        ((a.lb[1] >= this->lb[1] && a.lb[1] <= this->ub[1]) || (this->lb[1] >= a.lb[1] && this->lb[1] <= a.ub[1])) &&
        ((a.lb[2] >= this->lb[2] && a.lb[2] <= this->ub[2]) || (this->lb[2] >= a.lb[2] && this->lb[2] <= a.ub[2]));

}

float AABB::diagonalLength() const
{
    return (ub - lb).norm();
}

bool AABB::rayIntersection(const Ray& ray, float& t_in, float& t_out) const
{
    float dir_frac_x = (ray.direction[0] == 0.0) ? 1.0e32 : 1.0f / ray.direction[0];
    float dir_frac_y = (ray.direction[1] == 0.0) ? 1.0e32 : 1.0f / ray.direction[1];
    float dir_frac_z = (ray.direction[2] == 0.0) ? 1.0e32 : 1.0f / ray.direction[2];

    float tx1 = (lb[0] - ray.origin[0]) * dir_frac_x;
    float tx2 = (ub[0] - ray.origin[0]) * dir_frac_x;
    float ty1 = (lb[1] - ray.origin[1]) * dir_frac_y;
    float ty2 = (ub[1] - ray.origin[1]) * dir_frac_y;
    float tz1 = (lb[2] - ray.origin[2]) * dir_frac_z;
    float tz2 = (ub[2] - ray.origin[2]) * dir_frac_z;

    t_in = std::max(std::max(std::min(tx1, tx2), std::min(ty1, ty2)), std::min(tz1, tz2));
    t_out = std::min(std::min(std::max(tx1, tx2), std::max(ty1, ty2)), std::max(tz1, tz2));

    /* When t_out < 0 and the ray is intersecting with AABB, the whole AABB is behind us */
    if (t_out < 0)
    {
        return false;
    }

    return t_out >= t_in;
}


/**
 * Grid class
 */

Grid::Grid()
    : N(0, 0, 0)
    , cell_size(0)
{
}

void Grid::create(const AABB& bb, unsigned int n, float lambda)
{
    bounding_box = bb;

    float dx = bb.getDist(0);
    float dy = bb.getDist(1);
    float dz = bb.getDist(2);
    float V = bb.getVolume();

    cell_size = std::cbrtf(V / (lambda * n));
    N(0) = std::ceil(dx / cell_size);
    N(1) = std::ceil(dy / cell_size);
    N(2) = std::ceil(dz / cell_size);

    for (int x = 0; x < N(0); x++)
    {
        for (int y = 0; y < N(1); y++)
        {
            for (int z = 0; z < N(2); z++)
            {
                cells.push_back(Cell());
            }
        }
    }
}

Cell* Grid::getCell(unsigned int x, unsigned int y, unsigned int z)
{
    if (x >= N(0) || y >= N(1) || z >= N(2))
    {
        return nullptr;
    }
    return &(cells[x * N(1) * N(2) + y * N(2) + z]);
}

void Grid::addTriangle(Eigen::Vector3i tri_indices, const AABB& tri_bb)
{
    Eigen::Vector3i min_cell = ((tri_bb.lb - bounding_box.lb) / cell_size).cast<int>();
    Eigen::Vector3i max_cell = ((tri_bb.ub - bounding_box.lb) / cell_size).cast<int>();

    for (int x = min_cell.x(); x <= max_cell.x(); x++)
    {
        for (int y = min_cell.y(); y <= max_cell.y(); y++)
        {
            for (int z = min_cell.z(); z <= max_cell.z(); z++)
            {
                getCell(x, y, z)->push_back(tri_indices);
            }
        }
    }
}

void Grid::reset()
{
    for (int x = 0; x <= N(0); x++)
    {
        for (int y = 0; y <= N(1); y++)
        {
            for (int z = 0; z <= N(2); z++)
            {
                getCell(x, y, z)->clear();
            }
        }
    }
}
