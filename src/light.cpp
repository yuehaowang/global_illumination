#include <utility>
#include "light.hpp"
#include "geometry.hpp"
#include "utils.hpp"


/**
 * Light class
 */

Light::Light(Eigen::Vector3f pos, Eigen::Vector3f power)
    : position(pos)
    , radiance(power)
{
}

Eigen::Vector3f Light::getPosition() const
{
    return position;
}

Eigen::Vector3f Light::getRadiance() const
{
    return radiance;
}


/**
 * AreaLight class
 */

AreaLight::AreaLight(Eigen::Vector3f pos, Eigen::Vector3f power, Eigen::Vector2f size)
    : Light(pos, power)
    , area_size(size)
    , geometry_delegation(
        pos - Eigen::Vector3f(size[0], 0, size[1]) / 2,
        Eigen::Vector3f(size[0], 0, 0),
        Eigen::Vector3f(0, 0, size[1]),
        Eigen::Vector3f(0, -1, 0), nullptr)
{
}

Eigen::Vector3f AreaLight::emission(Eigen::Vector3f pos, Eigen::Vector3f dir)
{
    /* Retrieve geometric information, e.g., normal, at the given position */
    Interaction geom_it;
    if (!geometry_delegation.rayIntersection(geom_it, Ray(pos, dir, -DELTA)))
    {
        return Eigen::Vector3f(0, 0, 0);
    }

    return geom_it.normal.dot(dir) > 0 ? radiance : Eigen::Vector3f(0, 0, 0);
}

float AreaLight::samplePdf(const Interaction& ref_it, Eigen::Vector3f pos)
{
    Eigen::Vector3f diff = pos - ref_it.entry_point;
    Ray r(ref_it.entry_point, diff.normalized(), DELTA, diff.norm() + DELTA);

    Interaction light_it;

    if (!geometry_delegation.rayIntersection(light_it, r))
    {
        return 0;
    }

    if (-(ref_it.wi).dot(light_it.normal) < 0)
    {
        return 0;
    }
    
    return (1 / (area_size[0] * area_size[1])) * diff.squaredNorm() / abs(ref_it.wi.dot(light_it.normal));
}

Eigen::Vector3f AreaLight::sample(Interaction& ref_it, float* pdf)
{
    Eigen::Vector3f p0 = position - Eigen::Vector3f(area_size[0], 0, area_size[1]) / 2;

    float sample_x = mathutils::unif(0, area_size[0], 1)[0];
    float sample_z = mathutils::unif(0, area_size[1], 1)[0];
    Eigen::Vector3f sampled_light_pos = p0 + Eigen::Vector3f(sample_x, 0, sample_z);

    Eigen::Vector3f diff = sampled_light_pos - ref_it.entry_point;
    ref_it.wi = diff.normalized();

    /* Compute PDF of sampling a point in the area light */
    if (pdf)
    {
        *pdf = samplePdf(ref_it, sampled_light_pos);
    }

    return sampled_light_pos;
}

bool AreaLight::rayIntersection(Interaction& interaction, const Ray& ray)
{
    bool intersection = geometry_delegation.rayIntersection(interaction, ray);
    interaction.type = Interaction::Type::LIGHT;
    return intersection;
}
