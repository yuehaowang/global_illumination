#include <random>
#include <algorithm>
#include "utils.hpp"
#include "constant.hpp"


/**
 * strutils namespace
 */

void strutils::ltrim(std::string& s)
{
   s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
}

void strutils::rtrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}

void strutils::trim(std::string& s)
{
    ltrim(s);
    rtrim(s);
}


/**
 * mathutils namespace
 */


float mathutils::clamp(float x, float lo, float hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

unsigned char mathutils::gamma_correction(float x)
{
	return (unsigned char)(pow(mathutils::clamp(x, 0.0, 1.0), 1 / 2.2) * 255);
}

int mathutils::sgnf(float val)
{
	return (0 < val) - (val < 0);
}

static std::default_random_engine random_generator;

std::vector<float> mathutils::unif(float a, float b, int N)
{
    std::vector<float> res;
    std::uniform_real_distribution<float> dis(a, b);

    for (int i = 0; i < N; i++)
    {
        res.push_back(dis(random_generator));
    }
    return res;
}

Eigen::Vector2f mathutils::disk(float r)
{
    std::vector<float> u = unif(0.0f, 1.0f, 2);

    return Eigen::Vector2f(r * sqrt(u[0]), 2 * PI * u[1]);
}

Eigen::Matrix3f mathutils::rot_from_two_vectors(Eigen::Vector3f src, Eigen::Vector3f des)
{
    return Eigen::Quaternionf::FromTwoVectors(src, des).toRotationMatrix();
}

Eigen::Matrix3f mathutils::rot_align_normal(Eigen::Vector3f n)
{
    return rot_from_two_vectors(Eigen::Vector3f(0.0f, 0.0f, 1.0f), n);
}

Eigen::Vector3f mathutils::reflect(Eigen::Vector3f v1, Eigen::Vector3f n)
{
    return 2 * n.dot(v1) * n - v1;
}

Eigen::Vector3f mathutils::refract(Eigen::Vector3f v1, Eigen::Vector3f n, float ior)
{
    float cos_theta_i = v1.dot(n); 
    float eta = 1 / ior;
    if (cos_theta_i < 0)
    {
        eta = 1 / eta;
        n *= -1;
        cos_theta_i *= -1;
    }
    float c = sqrt(1 - eta * eta * (1 - cos_theta_i * cos_theta_i));
    return c < 0 ? Eigen::Vector3f(0, 0, 0) : (eta * (-v1) + (eta * cos_theta_i - c) * n);
}

float mathutils::power_heuristic(int n_f, float p_f, int n_g, float p_g, float beta)
{
    float c_f = (float)n_f * p_f, c_g = (float)n_g * p_g;
    return pow(c_f, beta) / (pow(c_f, beta) + pow(c_g, beta));
}
