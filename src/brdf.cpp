#include <vector>
#include "Eigen/Dense"
#include "interaction.hpp"
#include "utils.hpp"
#include "constant.hpp"
#include "brdf.hpp"


/**
 * BRDF class
 */

BRDF::BRDF()
{
}


/**
 * IdealDiffusion class
 */

IdealDiffusion::IdealDiffusion(Eigen::Vector3f diff)
    : reflectivity(diff)
{
}
    
Eigen::Vector3f IdealDiffusion::eval(const Interaction& interact)
{
    /* To satisfy the conservation of energy, BRDF <= 1 / PI */
    return reflectivity * PI_INV;
}

float IdealDiffusion::samplePdf(const Interaction& interact)
{
    float cos_theta_i = interact.wi.dot(interact.normal);
    return abs(cos_theta_i) * PI_INV;
}

float IdealDiffusion::sample(Interaction& interact)
{
    Eigen::Vector2f d = mathutils::disk(1.0f);
    float sin_theta = d[0], cos_theta = sqrt(1 - (sin_theta * sin_theta));
    float cos_phi = cos(d[1]), sin_phi = sin(d[1]);

    Eigen::Vector3f wi(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
    Eigen::Matrix3f rot = mathutils::rot_align_normal(interact.normal);
    interact.wi = rot * wi;

    return abs(cos_theta * PI_INV);
}

bool IdealDiffusion::isDelta() const
{
    return false;
}


/**
 * IdealSpecular class
 */

IdealSpecular::IdealSpecular(Eigen::Vector3f spec)
    : reflectivity(spec)
{
}

Eigen::Vector3f IdealSpecular::eval(const Interaction& interact)
{
    bool is_reflect = samplePdf(interact) > 0.0f;
    return is_reflect ? reflectivity : Eigen::Vector3f(0, 0, 0);
}

float IdealSpecular::samplePdf(const Interaction& interact)
{
    bool is_reflect = false;
    Eigen::Vector3f v2 = mathutils::reflect(interact.wo, interact.normal);
    if ((v2 - interact.wi).norm() < DELTA)
    {
        is_reflect = true;
    }
    return is_reflect ? 1.0f : 0.0f;
}

float IdealSpecular::sample(Interaction& interact)
{
    interact.wi = mathutils::reflect(interact.wo, interact.normal);

    return 1.0f;
}

bool IdealSpecular::isDelta() const
{
    return true;
}


/**
 * IdealTransmission class
 */

IdealTransmission::IdealTransmission(Eigen::Vector3f reflect, float idx_refract)
    : reflectivity(reflect)
    , ior(idx_refract)
{

}

Eigen::Vector3f IdealTransmission::eval(const Interaction& interact)
{
    bool is_refract = samplePdf(interact) > 0.0f;
    return is_refract ? reflectivity : Eigen::Vector3f(0, 0, 0);
}

float IdealTransmission::samplePdf(const Interaction& interact)
{
    bool is_refract = false;
    Eigen::Vector3f v2 = mathutils::refract(interact.wo, interact.normal, ior);
    if ((v2 - interact.wi).norm() < DELTA)
    {
        is_refract = true;
    }
    return is_refract ? 1.0f : 0.0f;
}

float IdealTransmission::sample(Interaction& interact)
{
    interact.wi = mathutils::refract(interact.wo, interact.normal, ior);

    return 1.0f;
}

bool IdealTransmission::isDelta() const
{
    return true;
}
