#include <math.h>
#include "geometry.hpp"
#include "constant.hpp"
#include "utils.hpp"
#include "objloader.hpp"


/**
 * Geometry class
 */

void Geometry::setMaterial(BRDF* new_mat)
{
    material = new_mat;
}


/**
 * Parallelogram class
 */

Parallelogram::Parallelogram(Eigen::Vector3f p0, Eigen::Vector3f s0, Eigen::Vector3f s1, Eigen::Vector3f normal, BRDF* mat)
    : p0(p0)
    , normal(normal.normalized())
{
    s0_len = s0.norm();
    s1_len = s1.norm();
    this->s0 = s0.normalized();
    this->s1 = s1.normalized();

    setMaterial(mat);
    buildBoundingBox();
}

bool Parallelogram::rayIntersection(Interaction& interaction, const Ray& ray)
{
    if (ray.direction.dot(normal) == 0)
    {
        return false;
    }
    
    float t = (p0 - ray.origin).dot(normal) / ray.direction.dot(normal);
    Eigen::Vector3f p0_p = ray.getPoint(t) - p0;
    float q0 = p0_p.dot(s0) / s0_len;
    float q1 = p0_p.dot(s1) / s1_len;
    if (q0 >= 0 && q0 <= s0.norm() && q1 >= 0 && q1 <= s1.norm() && t >= ray.range_min && t <= ray.range_max)
    {
        interaction.entry_dist = t;
        interaction.exit_dist = t;
        interaction.normal = normal;
        interaction.entry_point = ray.getPoint(t);
        if (material != nullptr)
        {
            interaction.material = (void*)material;
        }
        interaction.type = Interaction::Type::GEOMETRY;
        return true;
    }
    return false;
}

void Parallelogram::buildBoundingBox()
{
    Eigen::Vector3f p1 = p0 + s0 + s1;
    bounding_box.lb = p0.cwiseMin(p1);
    bounding_box.ub = p0.cwiseMax(p1);
}


/**
 * Sphere class
 */

Sphere::Sphere(Eigen::Vector3f p0, float r, BRDF* mat)
    : p0(p0)
    , radius(r)
{
    setMaterial(mat);
    buildBoundingBox();
}

bool Sphere::rayIntersection(Interaction& interaction, const Ray& ray)
{
    float a = 1.0f;
    float b = 2 * ray.direction.dot(ray.origin - p0);
    float c = (ray.origin - p0).squaredNorm() - radius * radius;
    float delta = b * b - 4 * a * c;

    if (delta < 0)
    {
        return false;
    }

    float t0 = (-b - sqrt(delta)) / 2 * a;
    float t1 = (-b + sqrt(delta)) / 2 * a;

    if (t1 < 0)
    {
        return false;
    }
    else if (t0 < 0 && t1 >= 0)
    {
        t0 = t1;
    }

    if (t0 < ray.range_min || t0 > ray.range_max)
    {
        return false;
    }

    interaction.entry_dist = t0;
    interaction.exit_dist = t1;
    interaction.entry_point = ray.getPoint(t0);
    Eigen::Vector3f r_vec = interaction.entry_point - p0;
    interaction.normal = r_vec.normalized();
    if (material != nullptr)
    {
        interaction.material = (void*)material;
    }
    interaction.type = Interaction::Type::GEOMETRY;

    return true;
}

void Sphere::buildBoundingBox()
{
    bounding_box = AABB(p0, radius);
}


/**
 * TriangleMesh class
 */

TriangleMesh::TriangleMesh(std::string file_path, BRDF* mat)
{
    setMaterial(mat);

    std::vector<Eigen::Vector2f> out_uvs;
    std::vector<int> out_vt_index;
    loadObj(file_path.c_str(), vertices, out_uvs, normals, vertices_index, out_vt_index, normals_index);
    
    num_triangles = vertices_index.size() / 3;

    has_grid = false;

    buildBoundingBox();
}

bool TriangleMesh::raySingleTriangleIntersection(Interaction& interaction, const Ray& ray, int v0_idx, int v1_idx, int v2_idx) const
{
    /* Vertices and normals of the triangle */
    Eigen::Vector3f v0 = vertices[vertices_index[v0_idx]];
    Eigen::Vector3f v1 = vertices[vertices_index[v1_idx]];
    Eigen::Vector3f v2 = vertices[vertices_index[v2_idx]];

    /* Position and direction of the ray */
    Eigen::Vector3f pos = ray.origin;
    Eigen::Vector3f dir = ray.direction;

    /* Compute barycentric coordinates */
    Eigen::Vector3f n = (v1 - v0).cross(v2 - v0);
    float d = (-dir).dot(n);
    Eigen::Vector3f e = (-dir).cross(pos - v0);

    if (fabs(d) > EPSILON)
    {
        float t = (pos - v0).dot(n) / d;
        
        if (t >= ray.range_min && t <= ray.range_max)
        {
            float v = (v2 - v0).dot(e) / d;
            float w = -(v1 - v0).dot(e) / d;

            if (v >= 0 && w >= 0 && v + w <= 1)
            {
                Eigen::Vector3f n0 = normals[normals_index[v0_idx]];
                Eigen::Vector3f n1 = normals[normals_index[v1_idx]];
                Eigen::Vector3f n2 = normals[normals_index[v2_idx]];

                interaction.entry_dist = t;
                interaction.entry_point = ray.getPoint(t);
                interaction.normal = (n0 + v * (n1 - n0) + w * (n2 - n0)).normalized();

                return true;
            }
        }
    }

    return false; 
}

bool TriangleMesh::rayIntersection(Interaction& interaction, const Ray& ray)
{
    Interaction final_interaction;
    if (has_grid)
    {
        Interaction bbox_it;
        if (bounding_box.rayIntersection(ray, bbox_it.entry_dist, bbox_it.exit_dist))
        {
            float cell_size = uniform_grid.cell_size;
            Eigen::Vector3f dir = ray.direction;

            /* The point where the ray enters the bounding box */
            Eigen::Vector3f entry_pt = ray.getPoint(bbox_it.entry_dist);
            /* Ray's position relative to the lower bound */
            Eigen::Vector3f grid_pt = entry_pt - bounding_box.lb;
            /* Cell coordinate of the ray */
            Eigen::Vector3i grid_cell = (grid_pt / cell_size).cast<int>();
            /* Marching direction of the ray in the grid */
            Eigen::Vector3i step(mathutils::sgnf(dir.x()), mathutils::sgnf(dir.y()), mathutils::sgnf(dir.z()));

            /* Values of t at which the ray crosses the next voxel boundary along xyz, repectively */
            Eigen::Vector3f t_max(INF, INF, INF);
            if (abs(dir.x()) > EPSILON)
            {
                t_max(0) = (step.x() < 0 ? (grid_cell.x() * cell_size - grid_pt.x()) : ((grid_cell.x() + step.x()) * cell_size - grid_pt.x())) / dir.x();
            }
            if (abs(dir.y()) > EPSILON)
            {
                t_max(1) = (step.y() < 0 ? (grid_cell.y() * cell_size - grid_pt.y()) : ((grid_cell.y() + step.y()) * cell_size - grid_pt.y())) / dir.y();
            }
            if (abs(dir.z()) > EPSILON)
            {
                t_max(2) = (step.z() < 0 ? (grid_cell.z() * cell_size - grid_pt.z()) : ((grid_cell.z() + step.z()) * cell_size - grid_pt.z())) / dir.z();
            }

            /* Values of t to cover a voxel along xyz, repectively */
            Eigen::Vector3f t_delta(
                (cell_size / dir.x()) * step.x(),
                (cell_size / dir.y()) * step.y(),
                (cell_size / dir.z()) * step.z()
            );

            Cell* target_cell = uniform_grid.getCell(grid_cell.x(), grid_cell.y(), grid_cell.z());

            if (target_cell)
            {
                /* Traverse cells along the ray */
                while (true)
                {
                    if (target_cell && target_cell->size() > 0)
                    {
                        /* Detect intersection of triangles in the current cell */
                        for (int j = 0; j < target_cell->size(); j++)
                        {
                            Eigen::Vector3i vert_idx = (*target_cell)[j];
                            Interaction cur_it;
                            if (raySingleTriangleIntersection(cur_it, ray, vert_idx.x(), vert_idx.y(), vert_idx.z()))
                            {
                                if (final_interaction.entry_dist == -1 || cur_it.entry_dist < final_interaction.entry_dist)
                                {
                                    final_interaction = cur_it;
                                }
                            }
                        }
                        if (final_interaction.entry_dist != -1)
                        {
                            break;
                        }
                    }

                    Eigen::Vector3f::Index k;
                    t_max.minCoeff(&k);
                    t_max(k) += t_delta(k);
                    grid_cell(k) += step(k);

                    if(grid_cell(k) < 0 || grid_cell(k) >= uniform_grid.N(k))
                    {
                        break;
                    }

                    target_cell = uniform_grid.getCell(grid_cell.x(), grid_cell.y(), grid_cell.z());
                }
            }
        }

    } else {
        for (int i = 0; i < num_triangles; i++)
        {
            Interaction cur_it;
            if (raySingleTriangleIntersection(cur_it, ray, 3 * i, 3 * i + 1, 3 * i + 2))
            {
                if (final_interaction.entry_dist == -1 || cur_it.entry_dist < final_interaction.entry_dist)
                {
                    final_interaction = cur_it;
                }
            }
        }
    }

    if (final_interaction.entry_dist != -1)
    {
        interaction = final_interaction;
        if (material != nullptr)
        {
            interaction.material = material;
        }
        interaction.type = Interaction::Type::GEOMETRY;

        return true;
    }

    return false;
}

void TriangleMesh::buildBoundingBox()
{

    bounding_box.lb = vertices[0].cwiseMin(vertices[1]);
    bounding_box.ub = vertices[0].cwiseMax(vertices[1]);
    for (int i = 2; i < vertices.size(); i++)
    {
        bounding_box.lb = bounding_box.lb.cwiseMin(vertices[i]);
        bounding_box.ub = bounding_box.ub.cwiseMax(vertices[i]);
    }

    if (has_grid)
    {
        uniform_grid.reset();
        buildUniformGrid();
    }
}

void TriangleMesh::applyTransformation(const Eigen::Affine3f& t)
{
    for (int i = 0; i < vertices.size(); i++)
    {
        vertices[i] = t * vertices[i];
    }

    Eigen::Matrix3f t_inv_tr = t.linear().inverse().transpose();
    for (int i = 0; i < normals.size(); i++)
    {
        normals[i] = (t_inv_tr * normals[i]).normalized();
    }

    buildBoundingBox();
}

void TriangleMesh::buildUniformGrid()
{
    uniform_grid.create(bounding_box, num_triangles);

    for (int i = 0; i < num_triangles; ++i)
    {
        Eigen::Vector3i vert_idx(3 * i, 3 * i + 1, 3 * i + 2);
        AABB tri_bb(
            vertices[vertices_index[vert_idx.x()]],
            vertices[vertices_index[vert_idx.y()]],
            vertices[vertices_index[vert_idx.z()]]
        );
        uniform_grid.addTriangle(vert_idx, tri_bb);
    }

    has_grid = true;
}