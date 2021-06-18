#pragma once
#include <iostream>
#include <string>
#include "Eigen/Dense"

#define UNREACHABLE std::cout << "Error: Unreachable code executed. Exit(-1)..." << std::endl; exit(-1);


/**
 * String processing utilities
 */

namespace strutils {
    /* Trim from left */
    void ltrim(std::string& s);
    /* Trim from right */
    void rtrim(std::string& s);
    /* Trim from both left and right */
    void trim(std::string& s);
};


/**
 * Mathematical utilities
 */

namespace mathutils {
    /* Clamps the input x to the closed range [lo, hi] */
    float clamp(float x, float lo, float hi);
    /* Performs Gamma correction on x and outputs an integer between 0-255. */
    unsigned char gamma_correction(float x);
    /* The sgn function for floats */
    int sgnf(float val);
    /* Uniformly sample N numbers among [a, b] */
    std::vector<float> unif(float a, float b, int N = 1);
    /* Uniformly sample a point on the 2D disk with radius of r */
    Eigen::Vector2f disk(float r);
    /* Find the rotation transforming the source vector to the destination vector */
    Eigen::Matrix3f rot_from_two_vectors(Eigen::Vector3f src, Eigen::Vector3f des);
    /* Find the transformation which aligns +z(0, 0, 1) to the given normal n */
    Eigen::Matrix3f rot_align_normal(Eigen::Vector3f n);
    /* Compute the reflected vector */
    Eigen::Vector3f reflect(Eigen::Vector3f v1, Eigen::Vector3f n);
    /* Compute the refracted vector */
    Eigen::Vector3f refract(Eigen::Vector3f v1, Eigen::Vector3f n, float ior = 1.0f);
    /* The power heuristic function for multiple importance sampling */
    float power_heuristic(int n_f, float p_f, int n_g, float p_g, float beta = 2);
};
